// SPDX-License-Identifier: Apache-2.0
// The external peer exercises real subprocess/pipe ownership, not browser rendering.
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { chmod, mkdtemp, readFile, readdir, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { test } from 'node:test';
import { chrome, waitFor } from './chrome_cdp.mjs';

const peer = join(dirname(fileURLToPath(import.meta.url)), 'cdp_peer.py');
async function alive(pid) {
  try {
    const status = await readFile(`/proc/${pid}/stat`, 'utf8');
    return status.slice(status.lastIndexOf(')') + 2).split(' ')[0] !== 'Z';
  } catch (error) { if (error.code === 'ENOENT') return false; throw error; }
}
async function fixture(run) {
  const directory = await mkdtemp(join(tmpdir(), 'cdp-contract-'));
  const previous = { CHROME: process.env.CHROME, TMPDIR: process.env.TMPDIR, CDP_PEER_RECEIPT: process.env.CDP_PEER_RECEIPT };
  process.env.CHROME = peer;
  process.env.TMPDIR = directory;
  process.env.CDP_PEER_RECEIPT = join(directory, 'receipt.json');
  const unrelated = spawn('/bin/sleep', ['60'], { detached: true, stdio: 'ignore' });
  console.log(`Controlled unrelated group: ${unrelated.pid}`);
  let receipt;
  try {
    await run({ directory, unrelated, async receipt() {
      receipt = JSON.parse(await readFile(process.env.CDP_PEER_RECEIPT, 'utf8'));
      return receipt;
    } });
  } finally {
    // Remove only a fixture-owned RED leak, never conceal the assertion above.
    if (!receipt) {
      try { receipt = JSON.parse(await readFile(process.env.CDP_PEER_RECEIPT, 'utf8')); } catch { /* spawn failed */ }
    }
    if (receipt) {
      for (const pid of [...receipt.children, receipt.pid]) {
        try { process.kill(pid, 'SIGKILL'); } catch (error) { if (error.code !== 'ESRCH') throw error; }
      }
    }
    unrelated.kill('SIGKILL');
    await new Promise(resolveExit => unrelated.once('exit', resolveExit));
    for (const [name, value] of Object.entries(previous)) {
      value === undefined ? delete process.env[name] : process.env[name] = value;
    }
    await chmod(directory, 0o700);
    await rm(directory, { recursive: true, force: true });
  }
}
async function assertClean(receipt, unrelated) {
  assert.equal(receipt.group, receipt.pid, 'helper owns a private group led by its spawned child');
  assert.equal(await alive(receipt.pid), false, 'owned startup process remains live');
  for (const pid of receipt.children) assert.equal(await alive(pid), false, 'owned descendant remains live');
  await assert.rejects(readFile(join(receipt.profile, 'missing')), error => error.code === 'ENOENT');
  assert.equal((await readdir(dirname(receipt.profile))).includes(receipt.profile.split('/').at(-1)), false, 'fresh profile remains');
  assert.equal(await alive(unrelated.pid), true, 'unrelated group must not be killed');
  console.log(`PASS owned cleanup: parent=${receipt.pid}, descendants=${receipt.children.join(',')}, profile absent, unrelated live`);
}

for (const [mode, method] of [['target-reject', 'Target.createTarget'], ['attach-reject', 'Target.attachToTarget']]) {
  test(`startup ${method} rejection cleans owned process, descendants and profile`, { timeout: 20000 }, async () => {
    await fixture(async ({ receipt, unrelated }) => {
      let failure;
      try { await chrome([`--peer=${mode}`]); } catch (error) { failure = error; }
      assert.ok(failure, 'startup must reject');
      assert.ok(failure.message.startsWith(`{"code":-32000,"message":"controlled ${method} rejection"}`), failure.message);
      await assertClean(await receipt(), unrelated);
      assert.match(failure.message, /Chrome startup diagnostics:/);
      assert.match(failure.message, /controlled peer startup diagnostic/);
      assert.match(failure.message, /"node":"v/);
      assert.match(failure.message, /"event":"write"/);
      assert.match(failure.message, /"event":"read"/);
    });
  });
}
test('startup timeout keeps original ten-second bound and cleans owned resources', { timeout: 20000 }, async () => {
  await fixture(async ({ receipt, unrelated }) => {
    const started = Date.now();
    await assert.rejects(chrome(['--peer=timeout']), error => {
      assert.ok(error.message.startsWith('CDP timed out: Target.createTarget;'), error.message);
      return true;
    });
    const elapsed = Date.now() - started;
    assert.ok(elapsed >= 9900 && elapsed < 14500, `elapsed=${elapsed}ms`);
    await assertClean(await receipt(), unrelated);
    console.log(`PASS unchanged startup deadline: ${elapsed}ms including bounded cleanup`);
  });
});
test('closed startup pipe rejects without unhandled errors and cleans owned resources', { timeout: 20000 }, async () => {
  await fixture(async ({ receipt, unrelated }) => {
    await assert.rejects(chrome(['--peer=pipe-close']), /Chrome.*pipe/i);
    await assertClean(await receipt(), unrelated);
  });
});
test('spawn failure preserves ENOENT and removes its fresh profile', { timeout: 20000 }, async () => {
  await fixture(async ({ directory, unrelated }) => {
    process.env.CHROME = resolve(directory, 'missing-browser');
    await assert.rejects(chrome(), error => error.code === 'ENOENT');
    assert.deepEqual((await readdir(directory)).filter(name => name.startsWith('fleet-dashboard-chrome-')), []);
    assert.equal(await alive(unrelated.pid), true);
  });
});
test('successful controlled initialization, evaluation and close preserve ownership', { timeout: 20000 }, async () => {
  await fixture(async ({ receipt, unrelated }) => {
    const browser = await chrome(['--peer=success']);
    const owned = await receipt();
    try {
      assert.equal(await browser.evaluate('private-expression-not-for-diagnostics'), 42);
      assert.equal(await alive(owned.pid), true);
      assert.ok(owned.arguments.includes('--remote-debugging-pipe'));
    } finally { await browser.close(); }
    await assertClean(owned, unrelated);
  });
});
test('startup diagnostics bound stderr and do not include raw CDP payloads', { timeout: 20000 }, async () => {
  await fixture(async ({ receipt, unrelated }) => {
    let failure;
    try { await chrome(['--peer=stderr-reject']); } catch (error) { failure = error; }
    assert.ok(failure);
    const diagnostic = JSON.parse(failure.message.split('\nChrome startup diagnostics: ')[1]);
    assert.ok(diagnostic.stderr.length <= 4000);
    assert.match(diagnostic.stderr, /DIAGNOSTIC-TAIL/);
    assert.ok(diagnostic.events.length <= 20);
    assert.equal(failure.message.includes('NO-RAW-CDP-PAYLOAD'), false);
    await assertClean(await receipt(), unrelated);
  });
});
test('startup rejection remains primary when fresh profile cleanup fails', { timeout: 20000 }, async () => {
  await fixture(async ({ receipt, unrelated }) => {
    let failure;
    try { await chrome(['--peer=cleanup-failure-reject']); } catch (error) { failure = error; }
    assert.ok(failure.message.startsWith('{"code":-32000,"message":"controlled Target.createTarget rejection"}'));
    const diagnostic = JSON.parse(failure.message.split('\nChrome startup diagnostics: ')[1]);
    assert.match(diagnostic.cleanupError, /EACCES/);
    const owned = await receipt();
    assert.equal(await alive(owned.pid), false);
    for (const pid of owned.children) assert.equal(await alive(pid), false);
    assert.equal(await alive(unrelated.pid), true);
    assert.equal((await readdir(dirname(owned.profile))).includes(owned.profile.split('/').at(-1)), true);
    // Restore only this deliberately unwritable fixture so final recovery can run.
    await chmod(dirname(owned.profile), 0o700);
  });
});
test('uncooperative owned startup group receives bounded kill escalation', { timeout: 20000 }, async () => {
  await fixture(async ({ receipt, unrelated }) => {
    const started = Date.now();
    let failure;
    try { await chrome(['--peer=ignore-term-reject']); } catch (error) { failure = error; }
    assert.ok(failure.message.startsWith('{"code":-32000,"message":"controlled Target.createTarget rejection"}'));
    assert.match(failure.message, /kill-owned-group/);
    assert.ok(Date.now() - started >= 1900 && Date.now() - started < 5000);
    await assertClean(await receipt(), unrelated);
  });
});
