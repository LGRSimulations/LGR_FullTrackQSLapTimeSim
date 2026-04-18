function initTabs() {
  const tabs = document.querySelectorAll('.tab');
  const sections = document.querySelectorAll('.tab-section');
  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      tabs.forEach(t => t.classList.remove('active'));
      sections.forEach(s => s.classList.remove('active'));
      tab.classList.add('active');
      document.getElementById(tab.dataset.tab).classList.add('active');
    });
  });
}

function parseNumber(value) {
  if (value === null || value === undefined || value === '') return null;
  const num = Number(value);
  return Number.isFinite(num) ? num : null;
}

async function loadMetadata() {
  const res = await fetch('/api/metadata');
  const data = await res.json();
  document.getElementById('scope').textContent = data.model_scope ?? '';
}

async function runLap() {
  const trackPath = document.getElementById('trackPath').value.trim();
  const mass = parseNumber(document.getElementById('massOverride').value);
  const aeroCp = parseNumber(document.getElementById('aeroCpOverride').value);

  const parameter_overrides = {};
  if (mass !== null) parameter_overrides.mass = mass;
  if (aeroCp !== null) parameter_overrides.aero_cp = aeroCp;

  const out = document.getElementById('lapOutput');
  out.textContent = 'Running...';

  const res = await fetch('/api/lap/run', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ parameter_overrides, track_file_path: trackPath || null }),
  });
  const data = await res.json();
  out.textContent = JSON.stringify(data, null, 2);
}

document.getElementById('runLapBtn').addEventListener('click', runLap);

initTabs();
loadMetadata();
