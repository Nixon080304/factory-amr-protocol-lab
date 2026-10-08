// SPDX-License-Identifier: Apache-2.0
// Chrome's debugging pipe avoids a dependency or an exposed debugging port.
import { spawn } from 'node:child_process';
import { mkdtemp, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

export async function chrome(extraArguments = []) {
  const profile = await mkdtemp(join(tmpdir(), 'fleet-dashboard-chrome-'));
  const process = spawn(globalThis.process.env.CHROME ?? '/usr/bin/google-chrome', [
    '--headless=new', '--no-sandbox', '--disable-dev-shm-usage',
    '--disable-background-networking', '--no-first-run', '--no-default-browser-check',
    '--remote-debugging-pipe', `--user-data-dir=${profile}`, ...extraArguments, 'about:blank',
  ], { stdio: ['ignore', 'ignore', 'pipe', 'pipe', 'pipe'] });
  let next = 0;
  let buffered = '';
  const pending = new Map();
  const listeners = [];
  let stderr = '';
  process.stderr.on('data', chunk => { stderr = (stderr + chunk).slice(-4000); });
  process.stdio[4].on('data', chunk => {
    buffered += chunk;
    let end;
    while ((end = buffered.indexOf('\0')) !== -1) {
      const message = JSON.parse(buffered.slice(0, end));
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
  process.on('exit', () => {
    for (const waiter of pending.values()) {
      clearTimeout(waiter.timer);
      waiter.reject(new Error(`Chrome exited: ${stderr}`));
    }
    pending.clear();
  });
  function send(method, params = {}, sessionId) {
    return new Promise((resolve, reject) => {
      const id = ++next;
      const timer = setTimeout(() => {
        pending.delete(id);
        reject(new Error(`CDP timed out: ${method}; ${stderr}`));
      }, 10000);
      pending.set(id, { resolve, reject, timer });
      process.stdio[3].write(JSON.stringify({ id, method, params, ...(sessionId ? { sessionId } : {}) }) + '\0');
    });
  }
  const { targetId } = await send('Target.createTarget', { url: 'about:blank' });
  const { sessionId } = await send('Target.attachToTarget', { targetId, flatten: true });
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
      if (process.exitCode === null) {
        await Promise.race([
          new Promise(resolve => process.once('exit', resolve)),
          new Promise(resolve => setTimeout(() => { process.kill('SIGKILL'); resolve(); }, 2000)),
        ]);
      }
      await rm(profile, { recursive: true, force: true });
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
