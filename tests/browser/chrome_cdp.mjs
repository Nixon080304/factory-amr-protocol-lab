// SPDX-License-Identifier: Apache-2.0
// Chrome's debugging pipe avoids a dependency or an exposed debugging port.
import { spawn } from 'node:child_process';
import { mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

export async function chrome(extraArguments = []) {
  const profile = await mkdtemp(join(tmpdir(), 'fleet-dashboard-chrome-'));
  const executable = globalThis.process.env.CHROME ?? '/usr/bin/google-chrome';
  const started = Date.now();
  const diagnostics = { node: globalThis.process.version, executable, browser: 'unmeasured', events: [] };
  function record(event, details = {}) {
    diagnostics.events.push({ ms: Date.now() - started, event, ...details });
    diagnostics.events = diagnostics.events.slice(-20);
  }
  const process = spawn(executable, [
    '--headless=new', '--no-sandbox', '--disable-dev-shm-usage',
    '--disable-background-networking', '--no-first-run', '--no-default-browser-check',
    '--remote-debugging-pipe', `--user-data-dir=${profile}`, ...extraArguments, 'about:blank',
  ], { stdio: ['ignore', 'ignore', 'pipe', 'pipe', 'pipe'], detached: true });
  diagnostics.pid = process.pid ?? null;
  console.log(`Chrome startup: ${JSON.stringify({ node: diagnostics.node, executable, pid: diagnostics.pid })}`);
  let next = 0;
  let buffered = '';
  const pending = new Map();
  const listeners = [];
  let stderr = '';
  let failure;
  let closing = false;
  function fail(error) {
    failure ??= error;
    for (const waiter of pending.values()) {
      clearTimeout(waiter.timer);
      waiter.reject(error);
    }
    pending.clear();
  }
  process.on('spawn', () => record('spawn'));
  process.on('error', error => { record('spawn-error', { code: error.code }); fail(error); });
  process.stderr.on('data', chunk => { stderr = (stderr + chunk).slice(-4000); });
  for (const [name, stream] of [['stderr', process.stderr], ['write', process.stdio[3]], ['read', process.stdio[4]]]) {
    stream.on('error', error => {
      record('pipe-error', { pipe: name, code: error.code });
      fail(new Error(`Chrome debugging pipe ${name} error: ${error.message}`));
    });
  }
  process.stdio[4].on('end', () => {
    record('pipe-end');
    if (!closing) fail(new Error('Chrome debugging pipe closed'));
  });
  process.stdio[4].on('data', chunk => {
    record('read', { bytes: chunk.length });
    buffered += chunk;
    let end;
    while ((end = buffered.indexOf('\0')) !== -1) {
      let message;
      try { message = JSON.parse(buffered.slice(0, end)); }
      catch { fail(new Error('Chrome debugging pipe returned invalid JSON')); return; }
      buffered = buffered.slice(end + 1);
      if (message.id) {
        const waiter = pending.get(message.id);
        if (waiter) {
          pending.delete(message.id);
          clearTimeout(waiter.timer);
          message.error ? waiter.reject(new Error(JSON.stringify(message.error))) : waiter.resolve(message.result);
        }
      } else {
        for (const listener of listeners) listener(message);
      }
    }
  });
  process.on('exit', (code, signal) => {
    record('exit', { code, signal });
    fail(new Error(`Chrome exited: ${stderr}`));
  });
  function send(method, params = {}, sessionId) {
    return new Promise((resolve, reject) => {
      if (failure) { reject(failure); return; }
      const id = ++next;
      const timer = setTimeout(() => {
        pending.delete(id);
        reject(new Error(`CDP timed out: ${method}; ${stderr}`));
      }, 10000);
      pending.set(id, { resolve, reject, timer });
      record('write', { method, id });
      process.stdio[3].write(JSON.stringify({ id, method, params, ...(sessionId ? { sessionId } : {}) }) + '\0', error => {
        record('write-complete', { id, code: error?.code ?? null });
        if (error) fail(new Error(`Chrome debugging pipe write error: ${error.message}`));
      });
    });
  }
  function groupExists() {
    // detached:true makes this spawned child the leader of its private group.
    // Never accept a caller-supplied PID/group or search unrelated processes.
    if (!process.pid) return false;
    try { globalThis.process.kill(-process.pid, 0); return true; }
    catch (error) { if (error.code === 'ESRCH') return false; throw error; }
  }
  function signalOwned(signal) {
    try { globalThis.process.kill(-process.pid, signal); }
    catch (error) { if (error.code !== 'ESRCH') throw error; }
  }
  async function cleanup(terminate = false) {
    closing = true;
    fail(new Error('Chrome closed'));
    let cleanupError;
    try {
      if (groupExists()) {
        if (terminate) { record('terminate-owned-group'); signalOwned('SIGTERM'); }
        const deadline = Date.now() + 2000;
        while (groupExists() && Date.now() < deadline) await new Promise(resolve => setTimeout(resolve, 40));
        if (groupExists()) {
          record('kill-owned-group');
          signalOwned('SIGKILL');
          await waitFor(() => !groupExists(), 'owned Chrome group exit', 2000);
        }
      }
    } catch (error) { cleanupError = error; }
    try { await rm(profile, { recursive: true, force: true }); }
    catch (error) { cleanupError ??= error; }
    if (cleanupError) throw cleanupError;
  }
  let sessionId;
  try {
    const { targetId } = await send('Target.createTarget', { url: 'about:blank' });
    ({ sessionId } = await send('Target.attachToTarget', { targetId, flatten: true }));
  } catch (error) {
    try { await cleanup(true); } catch (cleanupError) { diagnostics.cleanupError = cleanupError.message; }
    error.message += `\nChrome startup diagnostics: ${JSON.stringify({ ...diagnostics, stderr, exitCode: process.exitCode, signalCode: process.signalCode })}`;
    throw error;
  }
  // Version measurement is diagnostic only, never a new readiness wait.
  send('Browser.getVersion').then(version => {
    diagnostics.browser = version.product;
    console.log(`Chrome executed version: ${version.product}`);
  }).catch(() => { /* A failing diagnostic must not change successful startup. */ });
  return {
    send: (method, params) => send(method, params, sessionId),
    onEvent: listener => listeners.push(listener),
    async evaluate(expression) {
      const result = await send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true }, sessionId);
      if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
      return result.result.value;
    },
    async close() {
      try { await send('Browser.close'); } catch { /* Chrome closes the pipe itself. */ }
      await cleanup();
    },
  };
}

export async function waitFor(predicate, description, timeout = 10000) {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) {
    if (await predicate()) return;
    await new Promise(resolve => setTimeout(resolve, 40));
  }
  throw new Error(`Timed out: ${description}`);
}
