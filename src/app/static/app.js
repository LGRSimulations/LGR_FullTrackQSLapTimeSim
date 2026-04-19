const CHANNEL_REGISTRY = {
  speeds_kmh: { label: 'Speed',  unit: 'km/h', color: '#4e9af1', group: 'Speed',    flex: 2 },
  g_lat:      { label: 'Lat G',  unit: 'g',    color: '#f1a24e', group: 'G-Forces', flex: 1 },
  g_long:     { label: 'Long G', unit: 'g',    color: '#e05d5d', group: 'G-Forces', flex: 1 },
};

const VIZ_TABS = [
  { key: 'telemetry', label: 'Telemetry',   render: () => renderTelemetryTab() },
  { key: 'gg',        label: 'G-G Diagram', render: () => renderGGTab() },
];

let telemetryState = {
  runs: {},
  activeRunId: null,
  activePanes: ['speeds_kmh', 'g_lat', 'g_long'],
  activeTabKey: 'telemetry',
};

let _chartInstances = {};

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

function initInnerTabs() {
  document.querySelectorAll('.inner-tab').forEach(tab => {
    tab.addEventListener('click', () => {
      const panel = tab.closest('.panel');
      panel.querySelectorAll('.inner-tab').forEach(t => t.classList.remove('active'));
      panel.querySelectorAll('.inner-tab-section').forEach(s => s.classList.remove('active'));
      tab.classList.add('active');
      document.getElementById(tab.dataset.innerTab).classList.add('active');
    });
  });
}

function initVizPanel() {
  const list = document.getElementById('channelList');

  const listHeader = document.createElement('div');
  listHeader.className = 'channel-list-header';
  listHeader.textContent = 'Channels';
  list.appendChild(listHeader);

  const groups = {};
  Object.entries(CHANNEL_REGISTRY).forEach(([key, ch]) => {
    if (!groups[ch.group]) groups[ch.group] = [];
    groups[ch.group].push(key);
  });

  Object.entries(groups).forEach(([group, keys]) => {
    const groupHeader = document.createElement('div');
    groupHeader.className = 'channel-group-header';
    groupHeader.textContent = group;
    list.appendChild(groupHeader);

    keys.forEach(key => {
      const ch = CHANNEL_REGISTRY[key];
      const item = document.createElement('div');
      item.className = 'channel-item' + (telemetryState.activePanes.includes(key) ? '' : ' inactive');
      item.dataset.channelKey = key;
      item.innerHTML =
        `<div class="channel-dot" style="background:${ch.color}"></div>` +
        `<div><span class="channel-item-name">${ch.label}</span>` +
        `<span class="channel-item-unit">${ch.unit}</span></div>`;
      item.addEventListener('click', () => toggleChannel(key));
      list.appendChild(item);
    });
  });

  const tabBar = document.getElementById('vizTabBar');
  VIZ_TABS.forEach(tab => {
    const btn = document.createElement('button');
    btn.className = 'viz-tab' + (tab.key === telemetryState.activeTabKey ? ' active' : '');
    btn.textContent = tab.label;
    btn.addEventListener('click', () => {
      telemetryState.activeTabKey = tab.key;
      tabBar.querySelectorAll('.viz-tab').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      if (telemetryState.activeRunId) {
        tab.render();
      } else {
        document.getElementById('vizChartArea').innerHTML = '';
      }
    });
    tabBar.appendChild(btn);
  });
}

function storeTelemetry(data) {
  const runId = `run_${Date.now()}`;
  telemetryState.runs[runId] = {
    meta: {
      lap_time_s: data.lap_time_s,
      max_abs_g_lat: data.max_abs_g_lat,
      max_abs_g_long: data.max_abs_g_long,
      track_file_path: data.track_file_path,
    },
    telemetry: data.telemetry,
  };
  telemetryState.activeRunId = runId;
}

function renderViz() {
  const run = telemetryState.runs[telemetryState.activeRunId];
  if (!run) return;
  renderSummaryStrip(run.meta);
  const activeTab = VIZ_TABS.find(t => t.key === telemetryState.activeTabKey);
  if (activeTab) activeTab.render();
}

function renderSummaryStrip(meta) {
  document.getElementById('stat-lap-time').textContent = meta.lap_time_s.toFixed(2);
  document.getElementById('stat-max-lat-g').textContent = meta.max_abs_g_lat.toFixed(2);
  document.getElementById('stat-max-long-g').textContent = meta.max_abs_g_long.toFixed(2);
  const trackName = meta.track_file_path.split('/').pop().replace(/\.[^.]+$/, '');
  document.getElementById('stat-track').textContent = trackName;
  const status = document.getElementById('run-status');
  status.textContent = 'RUN OK';
  status.className = 'run-status ok';
  document.getElementById('csvDownloadBtn').disabled = false;
}

function renderTelemetryTab() {
  const run = telemetryState.runs[telemetryState.activeRunId];
  if (!run) return;

  const area = document.getElementById('vizChartArea');
  area.innerHTML = '';

  Object.keys(_chartInstances).forEach(k => {
    if (_chartInstances[k]) {
      _chartInstances[k].destroy();
      delete _chartInstances[k];
    }
  });

  const { telemetry } = run;

  telemetryState.activePanes.forEach(key => {
    const ch = CHANNEL_REGISTRY[key];
    if (!ch) return;

    const pane = document.createElement('div');
    pane.className = `chart-pane ${ch.flex === 2 ? 'chart-pane--lg' : 'chart-pane--sm'}`;
    pane.id = `pane-${key}`;

    const header = document.createElement('div');
    header.className = 'chart-pane-header';
    header.style.borderLeft = `3px solid ${ch.color}`;
    header.innerHTML =
      `<span class="chart-pane-title" style="color:${ch.color}">${ch.label}</span>` +
      `<span class="chart-pane-unit">${ch.unit}</span>`;

    const body = document.createElement('div');
    body.className = 'chart-pane-body';

    const canvas = document.createElement('canvas');
    canvas.id = `chart-${key}`;
    body.appendChild(canvas);

    pane.appendChild(header);
    pane.appendChild(body);
    area.appendChild(pane);

    _chartInstances[key] = new Chart(canvas, {
      type: 'line',
      data: {
        labels: telemetry.distances_m,
        datasets: [{
          data: telemetry[key],
          borderColor: ch.color,
          borderWidth: 1.5,
          pointRadius: 0,
          fill: false,
          tension: 0,
        }],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: false } },
        scales: {
          x: {
            type: 'linear',
            grid: { color: '#E8ECF0' },
            ticks: { color: '#A7AEB6', font: { size: 9 }, maxTicksLimit: 8 },
            title: { display: true, text: 'Distance (m)', color: '#A7AEB6', font: { size: 9 } },
          },
          y: {
            grid: { color: '#E8ECF0' },
            ticks: { color: '#A7AEB6', font: { size: 9 }, maxTicksLimit: 5 },
          },
        },
      },
    });
  });
}

function renderGGTab() {
  const run = telemetryState.runs[telemetryState.activeRunId];
  if (!run) return;

  const area = document.getElementById('vizChartArea');
  area.innerHTML = '';

  if (_chartInstances['gg']) {
    _chartInstances['gg'].destroy();
    delete _chartInstances['gg'];
  }

  const { g_lat, g_long } = run.telemetry;

  const container = document.createElement('div');
  container.className = 'gg-chart-container';
  const canvas = document.createElement('canvas');
  canvas.id = 'chart-gg';
  container.appendChild(canvas);
  area.appendChild(container);

  _chartInstances['gg'] = new Chart(canvas, {
    type: 'scatter',
    data: {
      datasets: [{
        data: g_lat.map((lat, i) => ({ x: lat, y: g_long[i] })),
        borderColor: '#4e9af1',
        backgroundColor: 'rgba(78,154,241,0.35)',
        pointRadius: 2,
        pointHoverRadius: 3,
      }],
    },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: {
          title: { display: true, text: 'Lat G', color: '#A7AEB6', font: { size: 10 } },
          grid: { color: '#E8ECF0' },
          ticks: { color: '#A7AEB6', font: { size: 9 } },
        },
        y: {
          title: { display: true, text: 'Long G', color: '#A7AEB6', font: { size: 10 } },
          grid: { color: '#E8ECF0' },
          ticks: { color: '#A7AEB6', font: { size: 9 } },
        },
      },
    },
  });
}

function toggleChannel(key) {
  const idx = telemetryState.activePanes.indexOf(key);
  if (idx === -1) {
    telemetryState.activePanes.push(key);
  } else {
    telemetryState.activePanes.splice(idx, 1);
  }
  document.querySelectorAll('.channel-item').forEach(el => {
    el.classList.toggle('inactive', !telemetryState.activePanes.includes(el.dataset.channelKey));
  });
  if (telemetryState.activeTabKey === 'telemetry' && telemetryState.activeRunId) {
    renderTelemetryTab();
  }
}

function downloadCSV() {
  const run = telemetryState.runs[telemetryState.activeRunId];
  if (!run) return;
  const { telemetry, meta } = run;

  const channelKeys = Object.keys(CHANNEL_REGISTRY);
  const headers = ['distance_m', ...channelKeys.map(k =>
    `${CHANNEL_REGISTRY[k].label.toLowerCase().replace(/\s+/g, '_')}_${CHANNEL_REGISTRY[k].unit.replace('/', '_per_')}`,
  )];

  const allKeys = ['distances_m', ...channelKeys];
  const n = telemetry.distances_m.length;
  const rows = [headers.join(',')];
  for (let i = 0; i < n; i++) {
    rows.push(allKeys.map(k => {
      const val = telemetry[k]?.[i];
      return val !== undefined ? Number(val).toFixed(4) : '';
    }).join(','));
  }

  const blob = new Blob([rows.join('\n')], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  const trackName = meta.track_file_path.split('/').pop().replace(/\.[^.]+$/, '');
  a.href = url;
  a.download = `lgr_lap_${trackName}_${meta.lap_time_s.toFixed(2)}s.csv`;
  a.click();
  URL.revokeObjectURL(url);
}

function populateParameters(p) {
  document.getElementById('p-general-name').value = p.general?.name ?? '';
  document.getElementById('p-general-mass').value = p.general?.mass ?? '';
  document.getElementById('p-general-base_mu').value = p.general?.base_mu ?? '';
  document.getElementById('p-aero-frontal_area').value = p.aerodynamics?.frontal_area ?? '';
  document.getElementById('p-aero-drag_coefficient').value = p.aerodynamics?.drag_coefficient ?? '';
  document.getElementById('p-aero-downforce_coefficient').value = p.aerodynamics?.downforce_coefficient ?? '';
  document.getElementById('p-aero-aero_cp').value = p.aerodynamics?.aero_cp ?? '';
  document.getElementById('p-geom-wheelbase').value = p.geometry?.wheelbase ?? '';
  document.getElementById('p-geom-front_track_width').value = p.geometry?.front_track_width ?? '';
  document.getElementById('p-geom-rear_track_width').value = p.geometry?.rear_track_width ?? '';
  document.getElementById('p-geom-cog_z').value = p.geometry?.cog_z ?? '';
  document.getElementById('p-geom-cog_longitudinal_pos').value = p.geometry?.cog_longitudinal_pos ?? '';
  document.getElementById('p-geom-max_cog_z').value = p.geometry?.max_cog_z ?? '';
  document.getElementById('p-vd-roll_stiffness').value = p.vehicle_dynamics?.roll_stiffness ?? '';
  document.getElementById('p-vd-suspension_stiffness').value = p.vehicle_dynamics?.suspension_stiffness ?? '';
  document.getElementById('p-vd-damping_coefficient').value = p.vehicle_dynamics?.damping_coefficient ?? '';
  document.getElementById('p-vd-max_roll_angle_deg').value = p.vehicle_dynamics?.max_roll_angle_deg ?? '';
  document.getElementById('p-dt-wheel_radius').value = p.drivetrain?.wheel_radius ?? '';
  document.getElementById('p-dt-final_drive_ratio').value = p.drivetrain?.final_drive_ratio ?? '';
  document.getElementById('p-dt-gear_ratios').value = (p.drivetrain?.gear_ratios ?? []).join(', ');
  document.getElementById('p-dt-transmission_efficiency').value = p.drivetrain?.transmission_efficiency ?? '';
}

function populateConfig(c) {
  document.getElementById('c-powertrain-path').value = c.powertrain?.powertrain ?? '';
  document.getElementById('c-powertrain-type').value = c.powertrain?.type ?? '';
  document.getElementById('c-track-file_path').value = c.track?.file_path ?? '';
  document.getElementById('c-tyre-file_path_longit').value = c.tyre_model?.file_path_longit ?? '';
  document.getElementById('c-tyre-file_path_lateral').value = c.tyre_model?.file_path_lateral ?? '';
  document.getElementById('c-tyre-type').value = c.tyre_model?.type ?? '';
  document.getElementById('c-sim-debug_mode').checked = c.debug_mode ?? false;
  document.getElementById('c-sim-full_telemetry_mode').checked = c.full_telemetry_mode ?? true;
  document.getElementById('c-ambient-air_density').value = c.ambient_conditions?.air_density ?? '';
}

async function loadParametersAndConfig() {
  try {
    const [pRes, cRes] = await Promise.all([
      fetch('/api/parameters'),
      fetch('/api/config'),
    ]);
    if (!pRes.ok || !cRes.ok) throw new Error('Failed to load defaults');
    populateParameters(await pRes.json());
    populateConfig(await cRes.json());
  } catch {
    const errorEl = document.getElementById('runError');
    if (errorEl) {
      errorEl.textContent = 'Could not load defaults. Check that the server is running.';
      errorEl.style.display = 'block';
    }
  }
}

function collectParameters() {
  const errors = [];

  function getFloat(id, label) {
    const el = document.getElementById(id);
    const val = parseFloat(el.value);
    if (isNaN(val)) {
      el.classList.add('field-error');
      errors.push(`${label} must be a number`);
    } else {
      el.classList.remove('field-error');
    }
    return val;
  }

  function getString(id) {
    const el = document.getElementById(id);
    el.classList.remove('field-error');
    return el.value.trim();
  }

  function getArray(id, label) {
    const el = document.getElementById(id);
    const parts = el.value.split(',').map(s => parseFloat(s.trim()));
    if (parts.some(isNaN)) {
      el.classList.add('field-error');
      errors.push(`${label} must be comma-separated numbers`);
    } else {
      el.classList.remove('field-error');
    }
    return parts;
  }

  const params = {
    general: {
      name: getString('p-general-name'),
      mass: getFloat('p-general-mass', 'Mass'),
      base_mu: getFloat('p-general-base_mu', 'Base mu'),
    },
    aerodynamics: {
      frontal_area: getFloat('p-aero-frontal_area', 'Frontal area'),
      drag_coefficient: getFloat('p-aero-drag_coefficient', 'Drag coefficient'),
      downforce_coefficient: getFloat('p-aero-downforce_coefficient', 'Downforce coefficient'),
      aero_cp: getFloat('p-aero-aero_cp', 'Aero CoP'),
    },
    geometry: {
      wheelbase: getFloat('p-geom-wheelbase', 'Wheelbase'),
      front_track_width: getFloat('p-geom-front_track_width', 'Front track width'),
      rear_track_width: getFloat('p-geom-rear_track_width', 'Rear track width'),
      cog_z: getFloat('p-geom-cog_z', 'CoG Z'),
      cog_longitudinal_pos: getFloat('p-geom-cog_longitudinal_pos', 'CoG longitudinal position'),
      max_cog_z: getFloat('p-geom-max_cog_z', 'Max CoG Z'),
    },
    vehicle_dynamics: {
      roll_stiffness: getFloat('p-vd-roll_stiffness', 'Roll stiffness'),
      suspension_stiffness: getFloat('p-vd-suspension_stiffness', 'Suspension stiffness'),
      damping_coefficient: getFloat('p-vd-damping_coefficient', 'Damping coefficient'),
      max_roll_angle_deg: getFloat('p-vd-max_roll_angle_deg', 'Max roll angle'),
    },
    drivetrain: {
      wheel_radius: getFloat('p-dt-wheel_radius', 'Wheel radius'),
      final_drive_ratio: getFloat('p-dt-final_drive_ratio', 'Final drive ratio'),
      gear_ratios: getArray('p-dt-gear_ratios', 'Gear ratios'),
      transmission_efficiency: getFloat('p-dt-transmission_efficiency', 'Transmission efficiency'),
    },
  };

  return { params, errors };
}

function collectConfig() {
  const errors = [];

  function getString(id) {
    const el = document.getElementById(id);
    if (el) el.classList.remove('field-error');
    return el ? el.value.trim() : '';
  }

  function requirePath(id, label) {
    const val = getString(id);
    if (!val) {
      document.getElementById(id).classList.add('field-error');
      errors.push(`${label} is required`);
    }
    return val;
  }

  function getFloat(id, label) {
    const el = document.getElementById(id);
    const val = parseFloat(el.value);
    if (isNaN(val)) {
      el.classList.add('field-error');
      errors.push(`${label} must be a number`);
    } else {
      el.classList.remove('field-error');
    }
    return val;
  }

  function getBool(id) {
    return document.getElementById(id).checked;
  }

  const cfg = {
    powertrain: {
      powertrain: requirePath('c-powertrain-path', 'Powertrain file'),
      type: getString('c-powertrain-type'),
    },
    track: {
      file_path: requirePath('c-track-file_path', 'Track file'),
    },
    tyre_model: {
      file_path_longit: requirePath('c-tyre-file_path_longit', 'Tyre longitudinal data file'),
      file_path_lateral: requirePath('c-tyre-file_path_lateral', 'Tyre lateral data file'),
      type: getString('c-tyre-type'),
    },
    debug_mode: getBool('c-sim-debug_mode'),
    full_telemetry_mode: getBool('c-sim-full_telemetry_mode'),
    ambient_conditions: {
      air_density: getFloat('c-ambient-air_density', 'Air density'),
    },
  };

  return { cfg, errors };
}

async function loadMetadata() {
  const res = await fetch('/api/metadata');
  const data = await res.json();
  const el = document.getElementById('scope');
  if (el) el.textContent = data.model_scope ?? '';
}

async function runLap() {
  const { params, errors: paramErrors } = collectParameters();
  const { cfg, errors: cfgErrors } = collectConfig();
  const allErrors = [...paramErrors, ...cfgErrors];

  const errorEl = document.getElementById('runError');

  if (allErrors.length > 0) {
    errorEl.textContent = allErrors.join('\n');
    errorEl.style.display = 'block';
    return;
  }
  errorEl.style.display = 'none';

  const status = document.getElementById('run-status');
  status.textContent = 'Running...';
  status.className = 'run-status';

  try {
    const res = await fetch('/api/lap/run', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ parameters: params, config: cfg }),
    });
    const data = await res.json();
    if (!res.ok) {
      status.textContent = 'RUN FAILED';
      status.className = 'run-status error';
      errorEl.textContent = data.detail ?? JSON.stringify(data, null, 2);
      errorEl.style.display = 'block';
    } else {
      storeTelemetry(data);
      renderViz();
    }
  } catch {
    status.textContent = 'RUN FAILED';
    status.className = 'run-status error';
    errorEl.textContent = 'Could not reach the server. Try again.';
    errorEl.style.display = 'block';
  }
}

// ── Lesson link handling ────────────────────────────────────────────────────

let _workspaceRoot = null;

async function getWorkspaceRoot() {
  if (_workspaceRoot) return _workspaceRoot;
  const res = await fetch('/api/workspace');
  const data = await res.json();
  _workspaceRoot = data.root;
  return _workspaceRoot;
}

function resolvePath(baseDir, rel) {
  const parts = [...baseDir.split('/'), ...rel.split('/')];
  const resolved = [];
  for (const p of parts) {
    if (p === '..') resolved.pop();
    else if (p !== '' && p !== '.') resolved.push(p);
  }
  return resolved.join('/');
}

function showToast(message) {
  let toast = document.getElementById('lgr-toast');
  if (!toast) {
    toast = document.createElement('div');
    toast.id = 'lgr-toast';
    document.body.appendChild(toast);
  }
  toast.textContent = message;
  toast.classList.add('visible');
  clearTimeout(toast._timer);
  toast._timer = setTimeout(() => toast.classList.remove('visible'), 4000);
}

function openVsCode(uri) {
  const iframe = document.createElement('iframe');
  iframe.style.cssText = 'display:none;position:fixed;';
  document.body.appendChild(iframe);

  let opened = false;
  const onBlur = () => { opened = true; };
  window.addEventListener('blur', onBlur, { once: true });

  iframe.src = uri;

  setTimeout(() => {
    window.removeEventListener('blur', onBlur);
    document.body.removeChild(iframe);
    if (!opened) {
      showToast('VS Code not detected. Download it at code.visualstudio.com to open this file.');
    }
  }, 1500);
}

async function processLessonLinks(container, lessonFile) {
  const workspaceRoot = await getWorkspaceRoot();
  const baseDir = 'docs/lessons';

  container.querySelectorAll('a[href]').forEach(link => {
    const href = link.getAttribute('href');
    if (!href || href.startsWith('http') || href.startsWith('#')) return;

    const resolved = resolvePath(baseDir, href);
    const isMd = resolved.endsWith('.md');

    if (isMd) {
      const isLesson = resolved.startsWith('docs/lessons/');
      link.classList.add('md-link');
      link.addEventListener('click', e => {
        e.preventDefault();
        if (isLesson) {
          const filename = resolved.split('/').pop();
          const item = document.querySelector(`.lesson-item[data-src="${filename}"]`);
          if (item) {
            item.click();
          } else {
            showToast('This lesson is not yet in the guide.');
          }
        } else {
          showToast('This document is not yet linked to a page.');
        }
      });
    } else {
      const absPath = workspaceRoot + '/' + resolved;
      const vscodeUri = 'vscode://file/' + absPath;
      link.classList.add('code-link');
      link.setAttribute('title', resolved);
      link.addEventListener('click', e => {
        e.preventDefault();
        openVsCode(vscodeUri);
      });
    }
  });
}

// ── Math pre-processing ──────────────────────────────────────────────────────
// Extract math before marked.js sees it to prevent _ and ^ mangling.

function renderMarkdownWithMath(md) {
  const stash = [];

  const save = (math, display) => {
    const key = `\x02MATH${stash.length}\x03`;
    stash.push({ key, math, display });
    return key;
  };

  const processed = md
    .replace(/\$\$([\s\S]*?)\$\$/g, (_, m) => save(m, true))
    .replace(/\$([^\$\n]+?)\$/g, (_, m) => save(m, false));

  let html = marked.parse(processed);

  for (const { key, math, display } of stash) {
    const rendered = katex.renderToString(math, { displayMode: display, throwOnError: false });
    html = html.replace(key, rendered);
  }

  return html;
}

// ── Lessons ─────────────────────────────────────────────────────────────────

async function loadLesson(filename, title) {
  const body = document.getElementById('lessonBody');
  const label = document.getElementById('lessonTitle');
  body.textContent = 'Loading...';
  label.textContent = title;
  const res = await fetch('/lessons/' + filename);
  const md = await res.text();
  body.innerHTML = renderMarkdownWithMath(md);
  body.querySelectorAll('img[src]').forEach(img => {
    const src = img.getAttribute('src');
    if (src && !src.startsWith('http') && !src.startsWith('/')) {
      img.setAttribute('src', '/lessons/' + src);
    }
  });
  processLessonLinks(body, filename);
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

// ── Chat Panel ───────────────────────────────────────────────────────────────

let chatHistory = [];

function positionChatPanel() {
  const tabBar = document.querySelector('.tab-bar');
  const top = tabBar.getBoundingClientRect().bottom;
  const panel = document.getElementById('chatPanel');
  panel.style.top = top + 'px';
  panel.style.height = `calc(100vh - ${top}px)`;
}

function toggleChatPanel(forceOpen) {
  const panel = document.getElementById('chatPanel');
  const btn = document.getElementById('askBtn');
  const open = forceOpen !== undefined ? forceOpen : panel.classList.contains('hidden');
  panel.classList.toggle('hidden', !open);
  btn.classList.toggle('active', open);
  btn.textContent = open ? 'Close' : 'Ask';
  if (open) document.getElementById('chatQuestion').focus();
}

function appendChatMessage(role, content, sources) {
  const thread = document.getElementById('chatMessages');
  const msg = document.createElement('div');
  msg.className = `chat-msg ${role}`;
  msg.textContent = content;

  if (sources && sources.length > 0) {
    const pillRow = document.createElement('div');
    pillRow.className = 'chat-sources';
    sources.forEach(src => {
      const pill = document.createElement('span');
      pill.className = 'chat-source-pill';
      pill.textContent = src.section;
      pill.title = src.file;
      pill.addEventListener('click', () => navigateToLesson(src.file));
      pillRow.appendChild(pill);
    });
    msg.appendChild(pillRow);
  }

  thread.appendChild(msg);
  thread.scrollTop = thread.scrollHeight;
  return msg;
}

function navigateToLesson(file) {
  const item = document.querySelector(`.lesson-item[data-src="${file}"]`);
  if (!item) {
    showToast('This lesson is not yet available in the guide.');
    return;
  }
  const lessonsTab = document.querySelector('.tab[data-tab="lessons"]');
  if (lessonsTab) lessonsTab.click();
  item.click();
}

async function sendChatMessage() {
  const input = document.getElementById('chatQuestion');
  const btn = document.getElementById('chatSubmit');
  const question = input.value.trim();
  if (!question) return;

  input.value = '';
  input.disabled = true;
  btn.disabled = true;

  appendChatMessage('user', question);
  const loading = (() => {
    const thread = document.getElementById('chatMessages');
    const el = document.createElement('div');
    el.className = 'chat-msg assistant chat-loading';
    el.innerHTML = '<span></span><span></span><span></span>';
    thread.appendChild(el);
    thread.scrollTop = thread.scrollHeight;
    return el;
  })();

  try {
    const res = await fetch('/api/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question, history: chatHistory }),
    });

    loading.remove();

    if (res.status === 503) {
      appendChatMessage('assistant', 'Chat is not configured. Contact your distributor for a key file.');
    } else if (!res.ok) {
      appendChatMessage('assistant', 'Something went wrong. Try again.');
    } else {
      const data = await res.json();
      appendChatMessage('assistant', data.answer, data.sources);
      chatHistory.push({ role: 'user', content: question });
      chatHistory.push({ role: 'assistant', content: data.answer });
    }
  } catch {
    loading.remove();
    appendChatMessage('assistant', 'Could not reach the server. Try again.');
  } finally {
    input.disabled = false;
    btn.disabled = false;
    input.focus();
  }
}

// ── Boot ────────────────────────────────────────────────────────────────────

document.getElementById('runLapBtn').addEventListener('click', runLap);
document.getElementById('csvDownloadBtn').addEventListener('click', downloadCSV);
document.getElementById('askBtn').addEventListener('click', () => toggleChatPanel());
document.getElementById('chatSubmit').addEventListener('click', sendChatMessage);
document.getElementById('chatQuestion').addEventListener('keydown', e => {
  if (e.key === 'Enter' && !e.target.disabled) sendChatMessage();
});

initTabs();
initInnerTabs();
initVizPanel();
initLessons();
loadMetadata();
loadParametersAndConfig();
positionChatPanel();
toggleChatPanel(true);
window.addEventListener('resize', positionChatPanel);
