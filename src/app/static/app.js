async function loadMetadata() {
  const res = await fetch('/api/metadata');
  const data = await res.json();
  document.getElementById('scope').textContent = data.model_scope;
  const ul = document.getElementById('deferredList');
  ul.innerHTML = '';
  for (const name of data.deferred_parameters || []) {
    const li = document.createElement('li');
    li.textContent = name;
    ul.appendChild(li);
  }
}

function parseNumber(value) {
  if (value === null || value === undefined || value === '') {
    return null;
  }
  const num = Number(value);
  return Number.isFinite(num) ? num : null;
}

async function runLap() {
  const trackPath = document.getElementById('trackPath').value.trim();
  const mass = parseNumber(document.getElementById('massOverride').value);
  const aeroCp = parseNumber(document.getElementById('aeroCpOverride').value);

  const parameter_overrides = {};
  if (mass !== null) parameter_overrides.mass = mass;
  if (aeroCp !== null) parameter_overrides.aero_cp = aeroCp;

  const body = {
    parameter_overrides,
    track_file_path: trackPath || null,
  };

  const out = document.getElementById('lapOutput');
  out.textContent = 'Running...';

  const res = await fetch('/api/lap/run', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await res.json();
  out.textContent = JSON.stringify(data, null, 2);
}

async function runLiftCoast() {
  const powers = document.getElementById('powerLimits').value
    .split(',')
    .map(s => Number(s.trim()))
    .filter(n => Number.isFinite(n));
  const energy = parseNumber(document.getElementById('energyTarget').value) ?? 0.5;

  const out = document.getElementById('liftOutput');
  out.textContent = 'Running...';

  const res = await fetch('/api/lift-coast/run', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ power_limits_kw: powers, energy_target_kwh: energy }),
  });
  const data = await res.json();
  out.textContent = JSON.stringify(data, null, 2);
}

document.getElementById('runLapBtn').addEventListener('click', runLap);
document.getElementById('runLiftBtn').addEventListener('click', runLiftCoast);

loadMetadata();
