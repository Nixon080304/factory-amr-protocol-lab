// SPDX-License-Identifier: Apache-2.0
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { mkdir, mkdtemp, writeFile, readFile } from 'node:fs/promises';
import { chrome, waitFor } from './chrome_cdp.mjs';

const fixture = spawn('.venv/bin/python3', ['tests/browser/dashboard_fixture.py'], { stdio: ['pipe', 'pipe', 'pipe'] });
let fixtureErrors = '';
fixture.stderr.on('data', chunk => { fixtureErrors += chunk; });
const lines = [];
createInterface({ input: fixture.stdout }).on('line', line => lines.push(JSON.parse(line)));
let browser;
const exceptions = [];
const consoleErrors = [];
const networkErrors = [];
const requests = [];
let expectedOutage = false;
await mkdir('artifacts/dashboard', { recursive: true });
const evidence = await mkdtemp('artifacts/dashboard/browser-');

async function command(command) {
  fixture.stdin.write(JSON.stringify({ command }) + '\n');
  await waitFor(() => lines.some(line => line.done === command), `fixture ${command}`);
}

try {
  const source = await readFile('src/fleet_manager/fleet_manager/static/app.js', 'utf8');
  const client = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
  const order = new client.SnapshotOrder();
  const value = { robots: [], missions: [], resources: [], dock_queue: [], events: [], updated_at: 1000, sequence: 10, session_id: 'session-one' };
  assert.equal(order.accept(value, true), 'new');
  assert.equal(order.accept({ ...value, sequence: 9 }), 'old');
  assert.equal(order.accept(value), 'duplicate');
  assert.equal(order.accept({ ...value, robots: {} }), 'invalid');
  assert.equal(order.accept({ ...value, resources: [{ resource_id: 'dock_01', kind: 2, waiters: 'bad' }] }), 'invalid');
  assert.equal(order.accept({ ...value, resources: [{ resource_id: 'dock_01', former_leases: [null] }] }), 'invalid');
  const claim = { robot_id: 'cart', mission_id: 'charge', resource_id: 'dock_01', lease_id: 'secret-token-a' };
  assert.equal(new client.SnapshotOrder().accept({ ...value, resources: [{ resource_id: 'dock_01', former_leases: [claim] }] }, true), 'invalid', 'client must reject leaked authority credentials');
  const fingerprintClaim = { robot_id: 'cart', mission_id: 'charge', resource_id: 'dock_01', lease_fingerprint: '0b0433472146' };
  assert.equal(new client.SnapshotOrder().accept({ ...value, resources: [{ resource_id: 'dock_01', former_leases: [fingerprintClaim] }] }, true), 'new');
  assert.equal(new client.SnapshotOrder().accept({ ...value, resources: [{ resource_id: 'dock_01', former_leases: [{ ...fingerprintClaim, lease_fingerprint: 123456789012 }] }] }, true), 'invalid', 'lease fingerprint must remain a typed string');
  assert.equal(order.accept({ ...value, dock_queue: [null] }), 'invalid');
  assert.equal(order.accept({ ...value, sequence: 11, robots: [{ robot_id: '<script>', battery_percent: 'broken' }] }), 'invalid');
  assert.equal(order.accept({ ...value, session_id: 'session-two', sequence: 1 }), 'refresh');
  assert.equal(order.accept({ ...value, session_id: 'session-two', sequence: 1 }, true), 'new');
  assert.equal(client.retryDelay(0, () => 0.5), 500);
  assert.equal(client.retryDelay(3, () => 0.5), 4000);
  assert.equal(client.retryDelay(100, () => 1), 8000);
  assert.ok(client.retryDelay(2, () => 0) < client.retryDelay(2, () => 1));
  const astral = '😀'.repeat(256);
  const boundedCases = [
    { robots: [{ robot_id: astral, health_detail: astral, unresolved_resources: [astral] }] },
    { resources: [{ resource_id: astral, kind: astral, owner: astral, waiters: [astral], unresolved_claimants: [astral], former_leases: [{ robot_id: astral, mission_id: astral, resource_id: astral, lease_fingerprint: 'a'.repeat(12) }] }] },
    { missions: [{ mission_id: astral, part: astral, pickup_station: astral, dropoff_station: astral }] },
    { dock_queue: [{ robot_id: astral, dock_id: astral, state: astral }] },
    { events: [{ event_id: astral, mission_id: astral, robot_id: astral, protocol: astral, event: astral, outcome: astral, detail: astral }] },
  ];
  assert.deepEqual(boundedCases.map(fields => new client.SnapshotOrder().accept({ ...value, ...fields }, true)), Array(5).fill('new'));
  assert.deepEqual([
    { robots: [{ robot_id: 'amr_01', health_detail: astral + '😀' }] },
    { events: [{ detail: astral + '😀' }] },
  ].map(fields => new client.SnapshotOrder().accept({ ...value, ...fields }, true)), ['invalid', 'invalid']);
  await waitFor(() => lines.some(line => line.ready) || fixture.exitCode !== null, 'production server startup');
  assert.equal(fixture.exitCode, null, fixtureErrors || 'dashboard fixture exited');
  const { url } = lines.find(line => line.ready);
  const initialCapture = (await (await fetch(`${url}/api/snapshot`)).json()).updated_at;
  assert.equal(typeof initialCapture, 'number');
  // Deliberately age the real initial capture beyond the three-second cutoff.
  // This is a freshness regression stimulus, not browser readiness or a retry.
  await waitFor(() => Date.now() / 1000 - initialCapture >= 3.2, 'intentional aged initial fixture capture');
  browser = await chrome();
  browser.onEvent(message => {
    if (message.method === 'Runtime.exceptionThrown') exceptions.push(message.params.exceptionDetails);
    if (message.method === 'Runtime.consoleAPICalled' && message.params.type === 'error') consoleErrors.push(message.params.args);
    if (message.method === 'Network.loadingFailed' && !expectedOutage) networkErrors.push(message.params.errorText);
    if (message.method === 'Network.requestWillBeSent') requests.push(message.params.request.url);
    if (message.method === 'Network.responseReceived' && message.params.response.status >= 400 && !expectedOutage) networkErrors.push(message.params.response);
  });
  await browser.send('Runtime.enable');
  await browser.send('Network.enable');
  await browser.send('Page.enable');
  await browser.send('Emulation.setDeviceMetricsOverride', { width: 1440, height: 1100, deviceScaleFactor: 1, mobile: false });
  await command('capture');
  await browser.send('Page.navigate', { url });
  await waitFor(() => browser.evaluate("document.querySelector('#connection')?.dataset.state === 'live'"), 'dashboard live');
  const renewedCapture = await browser.evaluate("fetch('/api/snapshot').then(response => response.json()).then(value => value.updated_at)");
  assert.ok(renewedCapture > initialCapture, 'acknowledged capture must replace the aged source timestamp');
  console.log(`PASS startup capture freshness: initial age ${(Date.now() / 1000 - initialCapture).toFixed(3)}s, renewed age ${(Date.now() / 1000 - renewedCapture).toFixed(3)}s`);
  const text = await browser.evaluate('document.body.innerText');
  assert.match(text, /amr_01/);
  assert.match(text, /amr_02/);
  assert.match(text, /dock_01/);
  assert.match(text, /CHARGING/);
  assert.match(text, /delivery-042/);
  assert.match(await browser.evaluate("document.querySelector('#dock .charge-value').textContent"), /26% \/ 67% target/);
  assert.equal(await browser.evaluate("fetch('/api/snapshot').then(response => response.json()).then(value => value.dock_queue[0].target_percent)"), 67);
  await command('unknown_target');
  await waitFor(() => browser.evaluate("document.querySelector('#dock .charge-value').textContent.includes('Unknown target')"), 'missing authoritative charging target');
  assert.equal(await browser.evaluate("document.querySelectorAll('[data-robot-id]').length"), 2);
  assert.equal(await browser.evaluate("document.querySelector('[data-resource-id=\"dock_01\"]').textContent.includes('amr_01')"), true);
  assert.equal(requests.findIndex(value => value.endsWith('/api/snapshot')) < requests.findIndex(value => value.endsWith('/api/events')), true);
  await command('update');
  await waitFor(() => browser.evaluate("document.querySelector('[data-robot-id=\"amr_02\"]').textContent.includes('WAITING_FOR_RESOURCE')"), 'live state transition');
  await mkdir(evidence, { recursive: true });
  const desktop = await browser.send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true });
  await writeFile(`${evidence}/desktop.png`, Buffer.from(desktop.data, 'base64'));
  expectedOutage = true;
  await command('stop');
  await waitFor(() => browser.evaluate("['stale', 'reconnecting'].includes(document.querySelector('#connection').dataset.state)"), 'visible stale/reconnecting state');
  assert.doesNotMatch(await browser.evaluate("document.querySelector('#connection').textContent"), /^Connected/);
  const stale = await browser.send('Page.captureScreenshot', { format: 'png' });
  await writeFile(`${evidence}/stale.png`, Buffer.from(stale.data, 'base64'));
  await command('restart');
  await waitFor(() => browser.evaluate("document.querySelector('#connection').dataset.state === 'live' && document.querySelector('[data-robot-id=\"amr_01\"]').textContent.includes('35%')"), 'fresh snapshot after reconnect', 15000);
  expectedOutage = false;
  assert.ok(requests.filter(value => value.endsWith('/api/snapshot')).length >= 2, 'reconnect must fetch snapshot again');
  assert.ok(requests.every(value => value.startsWith(url)), 'no external runtime requests');
  await browser.send('Emulation.setDeviceMetricsOverride', { width: 360, height: 800, deviceScaleFactor: 1, mobile: true });
  assert.equal(await browser.evaluate('document.documentElement.scrollWidth <= innerWidth'), true, 'phone overflow');
  const phone = await browser.send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true });
  await writeFile(`${evidence}/phone.png`, Buffer.from(phone.data, 'base64'));
  await command('quarantine');
  await waitFor(() => browser.evaluate("document.querySelector('[data-resource-id=\"central_aisle\"]').textContent.includes('Reconciliation required')"), 'quarantined resource evidence');
  assert.equal(await browser.evaluate("document.body.innerText.includes('old-aisle')"), false, 'raw lease credential must not enter rendered public media');
  assert.equal(await browser.evaluate("fetch('/api/snapshot').then(response => response.text()).then(body => body.includes('\\\"lease_id\\\"'))"), false, 'HTTP must not expose raw lease IDs');
  assert.equal(await browser.evaluate("document.querySelector('[data-resource-id=\"central_aisle\"]').textContent.includes('Waiters: amr_01')"), true, 'quarantine must retain visible waiters');
  assert.equal(await browser.evaluate("document.querySelector('[data-resource-id=\"central_aisle\"]').textContent.includes('Unresolved claims: amr_02, amr_01')"), true, 'all claimants must remain visible');
  assert.equal(await browser.evaluate("document.querySelector('[data-robot-id=\"amr_02\"]').textContent.includes('Unresolved: central_aisle')"), true, 'quarantine is not None held');
  assert.equal(await browser.evaluate("document.querySelector('[data-resource-id=\"central_aisle\"] strong').textContent"), 'No live authority', 'quarantine is not proof of unoccupied physical space');
  assert.equal(await browser.evaluate("Array.from(document.querySelector('[data-resource-id=\"central_aisle\"] .resource-state').querySelectorAll('span')).every((span, index, rows) => index === 0 || span.getBoundingClientRect().top >= rows[index - 1].getBoundingClientRect().bottom)"), true, 'waiters and claimant identities need separate readable lines');
  const quarantinePhone = await browser.send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true });
  await writeFile(`${evidence}/phone-quarantine.png`, Buffer.from(quarantinePhone.data, 'base64'));
  await browser.send('Emulation.setDeviceMetricsOverride', { width: 1440, height: 1100, deviceScaleFactor: 1, mobile: false });
  const quarantineDesktop = await browser.send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true });
  await writeFile(`${evidence}/desktop-quarantine.png`, Buffer.from(quarantineDesktop.data, 'base64'));
  await browser.send('Emulation.setDeviceMetricsOverride', { width: 360, height: 800, deviceScaleFactor: 1, mobile: true });
  await command('astral');
  await waitFor(() => browser.evaluate("document.querySelector('#connection').dataset.state === 'live' && document.querySelector('[data-robot-id=\"amr_02\"]').textContent.includes('😀'.repeat(256))"), 'astral observer snapshot stays live');
  assert.equal(await browser.evaluate("document.querySelectorAll('[data-resource-id]').length"), 1);
  assert.equal(await browser.evaluate("Array.from(document.querySelector('[data-resource-id]').dataset.resourceId).length"), 256);
  assert.equal(await browser.evaluate('document.documentElement.scrollWidth <= innerWidth'), true, 'astral phone overflow');
  assert.equal(await browser.evaluate("document.querySelector('#events').textContent.includes('<img src=x') && document.querySelector('#events img') === null && window.dashboardInjected !== true"), true, 'astral and markup stay inert textContent');
  await command('overcap');
  await waitFor(() => browser.evaluate("document.querySelector('#truncation').textContent.includes('former_leases: 1') && document.querySelector('#truncation').textContent.includes('waiters: 1') && document.querySelector('#truncation').textContent.includes('resources: 1')"), 'upstream cap counts remain visible');
  const incompleteRobot = await browser.evaluate("document.querySelector('[data-robot-id=\"amr_02\"]').textContent");
  assert.match(incompleteRobot, /Resource evidence incomplete/);
  assert.doesNotMatch(incompleteRobot, /None held/);
  assert.equal(await browser.evaluate('document.documentElement.scrollWidth <= innerWidth'), true, 'incomplete evidence phone overflow');
  const incompletePhone = await browser.send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true, clip: { x: 0, y: 0, width: 360, height: 1800, scale: 1 } });
  await writeFile(`${evidence}/phone-incomplete.png`, Buffer.from(incompletePhone.data, 'base64'));
  await browser.send('Emulation.setDeviceMetricsOverride', { width: 1440, height: 1100, deviceScaleFactor: 1, mobile: false });
  const incompleteDesktop = await browser.send('Page.captureScreenshot', { format: 'png' });
  await writeFile(`${evidence}/desktop-incomplete.png`, Buffer.from(incompleteDesktop.data, 'base64'));
  await browser.send('Emulation.setDeviceMetricsOverride', { width: 360, height: 800, deviceScaleFactor: 1, mobile: true });
  await command('dense');
  await waitFor(() => browser.evaluate("document.querySelectorAll('[data-resource-id]').length === 100"), 'dense waiter snapshot remains schema-valid');
  await command('truncated');
  await waitFor(() => browser.evaluate("document.querySelector('#truncation')?.textContent.includes('resources: 100')"), 'visible bounded snapshot truncation');
  assert.equal(await browser.evaluate('document.documentElement.scrollWidth <= innerWidth'), true, 'dense phone overflow');
  const truncatedPhone = await browser.send('Page.captureScreenshot', { format: 'png' });
  await writeFile(`${evidence}/phone-truncated.png`, Buffer.from(truncatedPhone.data, 'base64'));
  await waitFor(() => browser.evaluate("document.querySelector('#connection').dataset.state === 'stale'"), 'frozen ROS capture becomes stale despite SSE heartbeats');
  assert.equal(await browser.evaluate("document.querySelectorAll('h1').length === 1 && document.querySelector('main') !== null && document.querySelector('#connection').getAttribute('role') === 'status'"), true);
  assert.deepEqual(exceptions, [], 'uncaught browser exceptions');
  assert.deepEqual(consoleErrors, [], 'browser console errors');
  assert.deepEqual(networkErrors, [], 'unexpected browser network errors');
  const attacker = await chrome(['--host-resolver-rules=MAP dashboard.attacker.test 127.0.0.1', '--no-proxy-server']);
  try {
    const responses = [];
    attacker.onEvent(message => { if (message.method === 'Network.responseReceived') responses.push(message.params.response); });
    await attacker.send('Network.enable');
    await attacker.send('Page.enable');
    await attacker.send('Page.navigate', { url: url.replace('127.0.0.1', 'dashboard.attacker.test') + '/api/snapshot' });
    await waitFor(() => responses.length > 0, 'browser rebinding response');
    assert.equal(responses[0].status, 400, 'attacker origin must not read loopback snapshot');
    assert.ok(!responses.some(response => response.status === 200), 'no observer route exposed to attacker origin');
  } finally { await attacker.close(); }
  console.log(`PASS real Chrome: two robots, dock owner, SSE transition, stale/reconnect, fresh snapshot, desktop/360px phone, no console/network exceptions. Screenshots: ${evidence}`);
} finally {
  if (browser) await browser.close();
  if (fixture.exitCode === null) {
    fixture.stdin.end(JSON.stringify({ command: 'quit' }) + '\n');
    await Promise.race([
      new Promise(resolve => fixture.once('exit', resolve)),
      new Promise(resolve => setTimeout(() => { fixture.kill('SIGKILL'); resolve(); }, 2000)),
    ]);
  }
}
