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
  toggleChatPanel(false);
  const lessonsTab = document.querySelector('.tab[data-tab="lessons"]');
  if (lessonsTab) lessonsTab.click();
  const item = document.querySelector(`.lesson-item[data-src="${file}"]`);
  if (item) item.click();
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
  const loading = appendChatMessage('assistant', '...');

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
document.getElementById('chatClose').addEventListener('click', () => toggleChatPanel(false));
document.getElementById('chatSubmit').addEventListener('click', sendChatMessage);
document.getElementById('chatQuestion').addEventListener('keydown', e => {
  if (e.key === 'Enter') sendChatMessage();
});

initTabs();
initLessons();
loadMetadata();
positionChatPanel();
