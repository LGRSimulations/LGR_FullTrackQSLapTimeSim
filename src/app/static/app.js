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

async function loadLesson(filename, title) {
  const body = document.getElementById('lessonBody');
  const label = document.getElementById('lessonTitle');
  body.textContent = 'Loading...';
  label.textContent = title;
  const res = await fetch('/lessons/' + filename);
  const md = await res.text();
  body.innerHTML = marked.parse(md);
}

function initLessons() {
  const items = document.querySelectorAll('.lesson-item');
  items.forEach(item => {
    item.addEventListener('click', () => {
      items.forEach(i => i.classList.remove('active'));
      item.classList.add('active');
      loadLesson(item.dataset.src, item.textContent.trim());
    });
  });
  const first = document.querySelector('.lesson-item');
  if (first) loadLesson(first.dataset.src, first.textContent.trim());
}

document.getElementById('runLapBtn').addEventListener('click', runLap);

initTabs();
initLessons();
loadMetadata();
