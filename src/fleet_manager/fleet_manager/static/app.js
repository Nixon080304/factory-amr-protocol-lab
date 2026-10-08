// SPDX-License-Identifier: Apache-2.0
const lists = ['robots', 'missions', 'resources', 'dock_queue', 'events'];
const optionalString = value => value == null || typeof value === 'string';

export class SnapshotOrder {
  constructor() { this.session = null; this.sequence = -1; }
  accept(value, refresh = false) {
    if (!value || !lists.every(key => Array.isArray(value[key]) && value[key].length <= 100)
      || typeof value.session_id !== 'string' || value.session_id.length > 64
      || !Number.isSafeInteger(value.sequence) || value.sequence < 0
      || !(value.updated_at === null || Number.isFinite(value.updated_at))
      || !value.robots.every(robot => robot && typeof robot.robot_id === 'string'
        && robot.robot_id.length <= 256 && (robot.battery_percent == null ||
          (Number.isFinite(robot.battery_percent) && robot.battery_percent >= 0 && robot.battery_percent <= 100))
        && ['mode', 'health', 'payload_state', 'mission_id', 'health_detail', 'fault', 'current_resource'].every(key => optionalString(robot[key]))
        && (robot.pose == null || ['x', 'y', 'yaw'].every(key => Number.isFinite(robot.pose[key]))))
      || !value.resources.every(resource => resource && typeof resource.resource_id === 'string'
        && optionalString(resource.kind) && optionalString(resource.owner)
        && (resource.waiters == null || Array.isArray(resource.waiters) && resource.waiters.every(waiter => typeof waiter === 'string')))
      || !value.missions.every(mission => mission && typeof mission.mission_id === 'string'
        && ['state', 'assigned_robot_id', 'pickup_station', 'dropoff_station', 'part'].every(key => optionalString(mission[key])))
      || !value.dock_queue.every(charge => charge && typeof charge.robot_id === 'string' && typeof charge.dock_id === 'string' && typeof charge.state === 'string')
      || !value.events.every(event => event && typeof event === 'object')) return 'invalid';
    if (!refresh && this.session !== null && this.session !== value.session_id) return 'refresh';
    if (this.session === value.session_id && value.sequence < this.sequence) return 'old';
    if (this.session === value.session_id && value.sequence === this.sequence) return 'duplicate';
    this.session = value.session_id;
    this.sequence = value.sequence;
    return 'new';
  }
}

export function retryDelay(attempt, random = Math.random) {
  return Math.min(8000, Math.round(500 * 2 ** Math.min(attempt, 5) * (0.8 + random() * 0.4)));
}

function element(tag, text, className) {
  const item = document.createElement(tag);
  if (text !== undefined) item.textContent = String(text ?? '—');
  if (className) item.className = className;
  return item;
}

function fields(entries) {
  const list = element('dl');
  for (const [label, value] of entries) list.append(element('dt', label), element('dd', value ?? '—'));
  return list;
}

function render(value) {
  const robots = document.querySelector('#robots');
  robots.replaceChildren();
  for (const robot of value.robots) {
    const card = element('article', undefined, 'robot-card');
    card.dataset.robotId = robot.robot_id;
    const top = element('div', undefined, 'robot-top');
    top.append(element('h3', robot.robot_id), element('span', robot.mode ?? 'UNKNOWN', 'robot-mode'));
    const battery = element('div', undefined, 'battery-row');
    const percent = robot.battery_percent == null ? 'Unknown' : `${Math.round(robot.battery_percent)}%`;
    battery.append(element('strong', percent), element('span', 'MODELED BATTERY'));
    card.append(top, battery);
    if (robot.battery_percent != null) {
      const meter = element('meter');
      meter.min = 0; meter.max = 100; meter.low = 30; meter.high = 80; meter.optimum = 100; meter.value = robot.battery_percent;
      meter.setAttribute('aria-label', `${robot.robot_id} battery ${percent}`);
      card.append(meter);
    }
    const pose = robot.pose ? `${Number(robot.pose.x).toFixed(2)}, ${Number(robot.pose.y).toFixed(2)} m` : 'Unknown';
    card.append(fields([['HEALTH', robot.health ?? 'UNKNOWN'], ['ASSIGNMENT', robot.mission_id ?? 'Unassigned'], ['PAYLOAD', robot.payload_state ?? 'UNKNOWN'], ['RESOURCE', robot.current_resource ?? 'None held'], ['POSE', pose]]));
    if (robot.health_detail || robot.fault) card.append(element('p', robot.health_detail || robot.fault, 'health-detail'));
    robots.append(card);
  }
  const resources = document.querySelector('#resources');
  resources.replaceChildren();
  for (const resource of value.resources) {
    const row = element('div', undefined, 'resource-row');
    row.dataset.resourceId = resource.resource_id;
    const name = element('div', resource.resource_id, 'resource-name');
    name.append(element('small', resource.kind?.replaceAll('_', ' ') ?? 'resource'));
    const state = element('div', undefined, 'resource-state');
    state.append(element('strong', resource.owner ?? 'Unoccupied'));
    state.append(element('span', `Waiters: ${(resource.waiters ?? []).join(', ') || 'none'}`));
    if (resource.reconciliation_required) state.append(element('span', ' · Reconciliation required', 'uncertain'));
    row.append(name, state); resources.append(row);
  }
  const dockPanel = document.querySelector('#dock');
  dockPanel.replaceChildren();
  for (const dock of value.resources.filter(resource => resource.kind === 'dock')) {
    const owner = value.robots.find(robot => robot.robot_id === dock.owner);
    dockPanel.append(element('h3', dock.resource_id), element('p', `Owner: ${dock.owner ?? 'none'}`));
    if (owner) {
      dockPanel.append(element('p', `${owner.battery_percent == null ? 'Unknown' : Math.round(owner.battery_percent) + '%'} / 80% target`, 'charge-value'));
      dockPanel.append(element('p', `${owner.mode} · ${dock.reconciliation_required ? 'reconciliation required' : 'observed lease owner'}`));
    }
    const charges = value.dock_queue.filter(charge => charge.dock_id === dock.resource_id);
    const queue = element('ol');
    for (const charge of charges) queue.append(element('li', `${charge.robot_id} · ${charge.state}`));
    dockPanel.append(queue, element('p', charges.length ? `${charges.length} robot(s) in charging schedule` : `Dock waiters: ${(dock.waiters ?? []).join(', ') || 'none'}`));
  }
  if (!dockPanel.children.length) dockPanel.append(element('p', 'No docks configured.', 'empty'));
  const missions = document.querySelector('#missions');
  missions.replaceChildren();
  for (const mission of value.missions) {
    const row = element('div', undefined, 'mission-row');
    row.append(element('strong', mission.mission_id), element('span', `${mission.pickup_station} / ${mission.dropoff_station} · ${mission.part}`), element('span', mission.assigned_robot_id ?? 'Unassigned'), element('span', mission.state, 'mission-state'));
    missions.append(row);
  }
  if (!value.missions.length) missions.append(element('p', 'No missions recorded.', 'empty'));
  const events = document.querySelector('#events');
  events.replaceChildren();
  for (const event of [...value.events].reverse().slice(0, 50)) {
    const row = element('li');
    // Journal timestamps are process-monotonic evidence, not wall-clock dates.
    const time = element('span', event.protocol ? 'PROTOCOL' : `#${event.sequence ?? '—'}`, 'meta');
    row.append(time, element('span', `${event.robot_id ? event.robot_id + ' · ' : ''}${event.mission_id ?? ''} · ${event.state ?? event.event ?? 'Event'}${event.outcome ? ' · ' + event.outcome : ''}${event.detail ? ' · ' + event.detail : ''}`));
    events.append(row);
  }
  if (!value.events.length) events.append(element('li', 'No events received.', 'empty'));
  document.querySelector('#robot-count').textContent = value.robots.length;
  document.querySelector('#mission-count').textContent = value.missions.filter(mission => ['ASSIGNING', 'ASSIGNED', 'EXECUTING', 'REASSIGNING'].includes(mission.state)).length;
  document.querySelector('#fleet-state').textContent = value.fleet_state ?? 'UNKNOWN';
  document.querySelector('#sequence').textContent = `Snapshot #${value.sequence} · ${value.session_id.slice(0, 8)}`;
}

export function startDashboard() {
  const order = new SnapshotOrder();
  const connection = document.querySelector('#connection');
  let stream = null;
  let retry = null;
  let attempt = 0;
  let updatedAt = null;
  let stopped = false;
  let generation = 0;
  let controller = null;
  function status(state, text) { connection.dataset.state = state; connection.textContent = text; }
  function fresh() { return updatedAt !== null && Date.now() / 1000 - updatedAt < 3 && updatedAt <= Date.now() / 1000 + 5; }
  function fail() {
    if (stopped || retry !== null) return;
    generation += 1;
    stream?.close(); stream = null;
    controller?.abort();
    status('reconnecting', 'Reconnecting · cached state');
    retry = setTimeout(() => { retry = null; connect(); }, retryDelay(attempt++));
  }
  async function connect() {
    if (stopped) return;
    const current = ++generation;
    status(attempt ? 'reconnecting' : 'connecting', attempt ? 'Reconnecting · refreshing snapshot' : 'Connecting…');
    controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 3000);
    try {
      const response = await fetch('/api/snapshot', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) throw new Error('Snapshot unavailable');
      const snapshot = await response.json();
      if (current !== generation || stopped) return;
      const result = order.accept(snapshot, true);
      if (!['new', 'duplicate'].includes(result)) throw new Error('Malformed snapshot');
      updatedAt = snapshot.updated_at;
      render(snapshot);
      stream = new EventSource('/api/events');
      stream.addEventListener('snapshot', event => {
        if (current !== generation || stopped) return;
        try {
          const value = JSON.parse(event.data);
          const accepted = order.accept(value);
          if (accepted === 'old') return;
          if (accepted === 'invalid' || accepted === 'refresh') { fail(); return; }
          if (accepted === 'new') render(value);
          updatedAt = value.updated_at;
          if (fresh()) { attempt = 0; status('live', 'Connected · live observations'); }
          else status('stale', 'Stale · waiting for fleet update');
        } catch { fail(); }
      });
      stream.onerror = fail;
    } catch { if (current === generation) fail(); }
    finally { clearTimeout(timeout); }
  }
  const ageTimer = setInterval(() => {
    document.querySelector('#last-update').textContent = updatedAt === null ? 'Waiting for first snapshot' : `Last fleet update ${new Date(updatedAt * 1000).toLocaleTimeString()} · ${Math.max(0, Math.floor(Date.now() / 1000 - updatedAt))}s ago`;
    if (connection.dataset.state === 'live' && !fresh()) status('stale', 'Stale · waiting for fleet update');
  }, 250);
  function stop() { stopped = true; generation += 1; stream?.close(); controller?.abort(); clearTimeout(retry); clearInterval(ageTimer); }
  window.addEventListener('pagehide', stop, { once: true });
  connect();
  return stop;
}

if (typeof document !== 'undefined') startDashboard();
