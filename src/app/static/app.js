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

function parseNumber(value) {
  if (value === null || value === undefined || value === '') return null;
  const num = Number(value);
  return Number.isFinite(num) ? num : null;
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
  const [pRes, cRes] = await Promise.all([
    fetch('/api/parameters'),
    fetch('/api/config'),
  ]);
  const params = await pRes.json();
  const cfg = await cRes.json();
  populateParameters(params);
  populateConfig(cfg);
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
      powertrain: getString('c-powertrain-path'),
      type: getString('c-powertrain-type'),
    },
    track: {
      file_path: getString('c-track-file_path'),
    },
    tyre_model: {
      file_path_longit: getString('c-tyre-file_path_longit'),
      file_path_lateral: getString('c-tyre-file_path_lateral'),
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
  document.getElementById('scope').textContent = data.model_scope ?? '';
}

async function runLap() {
  const { params, errors: paramErrors } = collectParameters();
  const { cfg, errors: cfgErrors } = collectConfig();
  const allErrors = [...paramErrors, ...cfgErrors];

  const out = document.getElementById('lapOutput');

  if (allErrors.length > 0) {
    out.textContent = 'Fix input errors before running.\n\n' + allErrors.join('\n');
    return;
  }

  out.textContent = 'Running...';

  const res = await fetch('/api/lap/run', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ parameters: params, config: cfg }),
  });
  const data = await res.json();
  out.textContent = JSON.stringify(data, null, 2);
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
document.getElementById('askBtn').addEventListener('click', () => toggleChatPanel());
document.getElementById('chatSubmit').addEventListener('click', sendChatMessage);
document.getElementById('chatQuestion').addEventListener('keydown', e => {
  if (e.key === 'Enter' && !e.target.disabled) sendChatMessage();
});

initTabs();
initInnerTabs();
initLessons();
loadMetadata();
loadParametersAndConfig();
positionChatPanel();
toggleChatPanel(true);
window.addEventListener('resize', positionChatPanel);
