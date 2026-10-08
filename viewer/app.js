"use strict";

const state = {
  index: null,
  detail: null,
  episodes: [],
  filteredEpisodes: [],
  selectedKey: null,
  mode: "all",
  day: "all",
  search: "",
  currentFrame: 0,
  playing: false,
  speed: 1,
  playTimer: null,
  requestController: null,
  cameraLoadGeneration: 0,
};

const elements = {
  datasetRoot: document.querySelector("#datasetRoot"),
  serviceDot: document.querySelector("#serviceDot"),
  serviceStatus: document.querySelector("#serviceStatus"),
  episodeSearch: document.querySelector("#episodeSearch"),
  dayFilter: document.querySelector("#dayFilter"),
  resultCount: document.querySelector("#resultCount"),
  episodeList: document.querySelector("#episodeList"),
  emptyState: document.querySelector("#emptyState"),
  emptyMessage: document.querySelector("#emptyMessage"),
  detailView: document.querySelector("#detailView"),
  episodeDay: document.querySelector("#episodeDay"),
  episodeName: document.querySelector("#episodeName"),
  headingBadges: document.querySelector("#headingBadges"),
  metricStrip: document.querySelector("#metricStrip"),
  currentPhaseLabel: document.querySelector("#currentPhaseLabel"),
  phaseLegend: document.querySelector("#phaseLegend"),
  phaseTrackWrap: document.querySelector("#phaseTrackWrap"),
  phasePrior: document.querySelector("#phasePrior"),
  phaseSegments: document.querySelector("#phaseSegments"),
  phaseTrack: document.querySelector("#phaseTrack"),
  baseWindow: document.querySelector("#baseWindow"),
  graspMarker: document.querySelector("#graspMarker"),
  phasePlayhead: document.querySelector("#phasePlayhead"),
  phaseBreakdown: document.querySelector("#phaseBreakdown"),
  promptPanel: document.querySelector("#promptPanel"),
  currentSkill: document.querySelector("#currentSkill"),
  currentPromptVersion: document.querySelector("#currentPromptVersion"),
  currentFrameStatus: document.querySelector("#currentFrameStatus"),
  currentPrompt: document.querySelector("#currentPrompt"),
  frameTimestamp: document.querySelector("#frameTimestamp"),
  frameNumber: document.querySelector("#frameNumber"),
  frameTotal: document.querySelector("#frameTotal"),
  cameraGrid: document.querySelector("#cameraGrid"),
  frameSlider: document.querySelector("#frameSlider"),
  previousFrame: document.querySelector("#previousFrame"),
  nextFrame: document.querySelector("#nextFrame"),
  playPause: document.querySelector("#playPause"),
  activityChart: document.querySelector("#activityChart"),
  segmentSummary: document.querySelector("#segmentSummary"),
  segmentTableBody: document.querySelector("#segmentTableBody"),
  sidebar: document.querySelector("#sidebar"),
  sidebarToggle: document.querySelector("#sidebarToggle"),
  sidebarBackdrop: document.querySelector("#sidebarBackdrop"),
  toast: document.querySelector("#toast"),
};

function createIcons() {
  if (window.lucide) {
    window.lucide.createIcons({ attrs: { "stroke-width": 1.8 } });
  }
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function formatDuration(seconds) {
  const value = Number(seconds || 0);
  if (value < 60) return `${value.toFixed(1)} s`;
  const minutes = Math.floor(value / 60);
  return `${minutes}m ${(value - minutes * 60).toFixed(0)}s`;
}

function formatEpisodeTime(name) {
  const value = String(name).slice(0, 6);
  return /^\d{6}$/.test(value)
    ? `${value.slice(0, 2)}:${value.slice(2, 4)}:${value.slice(4, 6)}`
    : value;
}

function formatFrameTime(frame) {
  const milliseconds = Number(frame.timestampNs) / 1_000_000;
  const date = new Date(milliseconds);
  const time = new Intl.DateTimeFormat("zh-CN", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(date);
  return `${time}.${String(date.getMilliseconds()).padStart(3, "0")} · +${frame.relativeSeconds.toFixed(2)} s`;
}

function segmentClass(skillId) {
  const value = String(skillId).toLowerCase();
  if (value === "s1") return "s1";
  if (value === "s2") return "s2";
  if (value === "s3") return "s3";
  return "other";
}

function episodeHasSkill(episode, skillId) {
  const target = String(skillId).toUpperCase();
  return episode.segments.some((segment) => String(segment.skillId).toUpperCase() === target);
}

function episodeKey(episode) {
  return `${episode.day}/${episode.name}`;
}

function imageUrl(frameIndex, camera) {
  const summary = state.detail.summary;
  const query = new URLSearchParams({
    day: summary.day,
    episode: summary.name,
    frame: String(frameIndex),
    camera,
  });
  return `/api/image?${query}`;
}

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.remove("hidden");
  window.clearTimeout(showToast.timeout);
  showToast.timeout = window.setTimeout(() => elements.toast.classList.add("hidden"), 3500);
}

function setServiceStatus(type, text) {
  elements.serviceDot.classList.remove("online", "error");
  if (type) elements.serviceDot.classList.add(type);
  elements.serviceStatus.textContent = text;
}

function flattenEpisodes(index) {
  return index.days.flatMap((group) => group.episodes);
}

function renderDayFilter() {
  const days = [...new Set(state.episodes.map((episode) => episode.day))];
  elements.dayFilter.innerHTML = [
    '<option value="all">全部日期</option>',
    ...days.map((day) => `<option value="${escapeHtml(day)}">${escapeHtml(day)}</option>`),
  ].join("");
  elements.dayFilter.value = state.day;
}

function miniSegments(episode) {
  const total = Math.max(1, episode.segments.reduce((sum, item) => sum + item.frameCount, 0));
  return episode.segments
    .map(
      (segment) =>
        `<span class="${segmentClass(segment.skillId)}" style="width:${(
          (segment.frameCount / total) *
          100
        ).toFixed(3)}%"></span>`,
    )
    .join("");
}

function applyFilters() {
  const query = state.search.trim().toLowerCase();
  state.filteredEpisodes = state.episodes.filter((episode) => {
    if (state.mode === "paired" && !episode.paired) return false;
    if (state.mode === "s3" && !episodeHasSkill(episode, "S3")) return false;
    if (state.day !== "all" && episode.day !== state.day) return false;
    if (!query) return true;
    const promptText = episode.segments.map((segment) => segment.task).join(" ");
    const skills = episode.segments.map((segment) => segment.skillId).join(" ");
    return `${episode.name} ${episode.day} ${skills} ${promptText}`.toLowerCase().includes(query);
  });
  renderEpisodeList();
}

function renderEpisodeList() {
  elements.resultCount.textContent = `${state.filteredEpisodes.length} 条`;
  if (!state.filteredEpisodes.length) {
    elements.episodeList.innerHTML = '<div class="list-empty">没有匹配的数据</div>';
    return;
  }
  elements.episodeList.innerHTML = state.filteredEpisodes
    .map((episode) => {
      const active = episodeKey(episode) === state.selectedKey;
      const rate = `${(episode.validRate * 100).toFixed(1)}%`;
      const qualityClass = episode.completeCandidate ? "" : "warn";
      const icon = episode.completeCandidate ? "circle-check" : "triangle-alert";
      return `
        <button type="button" class="episode-item ${active ? "active" : ""}" data-key="${escapeHtml(
          episodeKey(episode),
        )}">
          <div class="episode-item-top">
            <span class="episode-time">${escapeHtml(formatEpisodeTime(episode.name))}</span>
            <i class="quality-icon ${qualityClass}" data-lucide="${icon}"></i>
          </div>
          <div class="episode-name-small">${escapeHtml(episode.name)}</div>
          <div class="mini-segments">${miniSegments(episode)}</div>
          <div class="episode-item-meta">
            <span>${episode.frameCount} 帧</span>
            <span>${formatDuration(episode.durationSeconds)}</span>
            <span>${rate} 有效</span>
          </div>
          <div class="episode-item-bottom">
            <span>${escapeHtml(episode.segments.map((segment) => segment.skillId).join(" → "))}</span>
            <span>${episode.repairCount} 修复</span>
          </div>
        </button>`;
    })
    .join("");
  elements.episodeList.querySelectorAll(".episode-item").forEach((button) => {
    button.addEventListener("click", () => {
      const episode = state.episodes.find((item) => episodeKey(item) === button.dataset.key);
      if (episode) selectEpisode(episode);
    });
  });
  createIcons();
}

function renderHeading() {
  const summary = state.detail.summary;
  const quality = state.detail.quality;
  elements.episodeDay.textContent = `${summary.day} · ${summary.workflow || "atomic episode"}`;
  elements.episodeName.textContent = summary.name;
  const badges = [
    `<span class="badge ${quality.completeCandidate ? "good" : "warn"}">${
      quality.completeCandidate ? "数据完整" : "需要检查"
    }</span>`,
    `<span class="badge">${summary.fps.toFixed(0)} Hz</span>`,
    `<span class="badge">${summary.paired ? "连续采集" : "原子技能"}</span>`,
  ];
  if (quality.repairCount) badges.push(`<span class="badge warn">${quality.repairCount} 帧修复</span>`);
  elements.headingBadges.innerHTML = badges.join("");
  elements.metricStrip.innerHTML = [
    [summary.frameCount.toLocaleString(), "总帧数"],
    [formatDuration(summary.durationSeconds), "记录时长"],
    [`${(summary.validRate * 100).toFixed(1)}%`, "有效帧"],
    [String(state.detail.segments.length), "Prompt 段"],
  ]
    .map(
      ([value, label]) =>
        `<div class="metric"><span class="metric-value">${escapeHtml(value)}</span><span class="metric-label">${escapeHtml(
          label,
        )}</span></div>`,
    )
    .join("");
}

function markerFrame(name, fallback) {
  const value = state.detail.phaseMarkers[name];
  return Number.isInteger(value?.frame) ? value.frame : fallback;
}

function renderPhaseTrack() {
  const detail = state.detail;
  const total = Math.max(1, detail.frames.length);
  const paired = detail.summary.paired;
  elements.phaseTrackWrap.classList.toggle("atomic", !paired);
  elements.phasePrior.classList.toggle("hidden", !paired);
  elements.phaseSegments.innerHTML = detail.segments
    .map(
      (segment) => `
        <div class="phase-segment ${segmentClass(segment.skillId)}" data-frame="${segment.startFrame}"
          style="width:${((segment.frameCount / total) * 100).toFixed(4)}%">
          <span>${escapeHtml(segment.skillId)} · ${segment.frameCount} 帧</span>
        </div>`,
    )
    .join("");
  elements.phaseSegments.querySelectorAll(".phase-segment").forEach((segment) => {
    segment.addEventListener("click", (event) => {
      event.stopPropagation();
      setFrame(Number(segment.dataset.frame));
    });
  });

  const graspFrame = markerFrame("s1_grasp_complete_ns", null);
  const b1Start = markerFrame("b1_start_ns", null);
  const b1Complete = markerFrame("b1_complete_ns", markerFrame("s2_start_ns", null));
  const legend = [];
  if (paired) legend.push(["b0", "B0 未记录"]);
  detail.segments.forEach((segment) => {
    legend.push([segmentClass(segment.skillId), segment.skillId]);
    if (
      String(segment.skillId).toUpperCase() === "S1" &&
      Number.isInteger(b1Start) &&
      Number.isInteger(b1Complete)
    ) {
      legend.push(["b1", "B1H"]);
    }
  });
  elements.phaseLegend.innerHTML = legend
    .map(
      ([className, label]) =>
        `<span><b class="legend-swatch ${className}"></b>${escapeHtml(label)}</span>`,
    )
    .join("");
  if (Number.isInteger(b1Start) && Number.isInteger(b1Complete) && b1Complete > b1Start) {
    elements.baseWindow.classList.remove("hidden");
    elements.baseWindow.style.left = `${(b1Start / total) * 100}%`;
    elements.baseWindow.style.width = `${((b1Complete - b1Start) / total) * 100}%`;
  } else {
    elements.baseWindow.classList.add("hidden");
  }
  if (Number.isInteger(graspFrame)) {
    elements.graspMarker.classList.remove("hidden");
    elements.graspMarker.style.left = `${(graspFrame / total) * 100}%`;
  } else {
    elements.graspMarker.classList.add("hidden");
  }

  const counts = [];
  if (Number.isInteger(graspFrame)) counts.push(["S1 抓取", graspFrame]);
  if (Number.isInteger(graspFrame) && Number.isInteger(b1Start)) {
    counts.push(["稳定抓持", Math.max(0, b1Start - graspFrame)]);
  }
  if (Number.isInteger(b1Start) && Number.isInteger(b1Complete)) {
    counts.push(["B1H", Math.max(0, b1Complete - b1Start)]);
  }
  if (Number.isInteger(b1Complete)) counts.push(["S2", Math.max(0, total - b1Complete)]);
  if (!counts.length) {
    detail.segments.forEach((segment) => counts.push([segment.skillId, segment.frameCount]));
  }
  elements.phaseBreakdown.innerHTML = counts
    .map(([label, count]) => `<span><b>${escapeHtml(label)}</b> ${count} 帧</span>`)
    .join("");
}

function renderCameraGrid() {
  elements.cameraGrid.innerHTML = state.detail.cameraNames
    .map(
      (camera) => `
        <figure class="camera-view" data-camera="${escapeHtml(camera.name)}">
          <div class="camera-image-wrap">
            <img alt="${escapeHtml(camera.label)}" />
            <span class="camera-unavailable hidden">当前帧无图像</span>
          </div>
          <figcaption><span>${escapeHtml(camera.label)}</span><span>${escapeHtml(camera.name)}</span></figcaption>
        </figure>`,
    )
    .join("");
}

function renderSegmentTable() {
  const fps = state.detail.summary.fps || 15;
  elements.segmentSummary.textContent = `${state.detail.segments.length} 个非重叠 segment · Prompt 来源 ${state.detail.summary.segmentSource}`;
  elements.segmentTableBody.innerHTML = state.detail.segments
    .map((segment) => {
      const base = segment.baseMotion?.allowed
        ? `${escapeHtml(segment.baseMotion.concurrent_prior || "允许")} · 不进标签`
        : "零速";
      return `
        <tr>
          <td><span class="table-skill ${segmentClass(segment.skillId)}">${escapeHtml(
            segment.skillId,
          )} <small>${escapeHtml(segment.promptVersion)}</small></span></td>
          <td class="table-prompt">${escapeHtml(segment.task)}</td>
          <td>${segment.startFrame}–${segment.endFrame}<br><span class="muted">${segment.frameCount} 帧</span></td>
          <td>${formatDuration(segment.frameCount / fps)}</td>
          <td>${base}</td>
        </tr>`;
    })
    .join("");
}

function currentPhase(frameIndex) {
  const grasp = markerFrame("s1_grasp_complete_ns", null);
  const b1Start = markerFrame("b1_start_ns", null);
  const b1Complete = markerFrame("b1_complete_ns", markerFrame("s2_start_ns", null));
  if (Number.isInteger(grasp) && frameIndex < grasp) return "S1 · 抓取柜门把手";
  if (Number.isInteger(b1Start) && frameIndex < b1Start) return "S1 · 稳定抓持";
  if (Number.isInteger(b1Complete) && frameIndex < b1Complete) return "B1H · 底盘后退 + 左臂调整";
  if (Number.isInteger(b1Complete)) return "S2 · 松手并用手背推门";
  const segment = state.detail.segments.find(
    (item) => frameIndex >= item.startFrame && frameIndex <= item.endFrame,
  );
  if (segment) return `${segment.skillId} · ${segment.task}`;
  return state.detail.frames[frameIndex]?.skillId || "未标注阶段";
}

function loadDecodedImage(url) {
  return new Promise((resolve, reject) => {
    const loader = new Image();
    loader.onload = async () => {
      try {
        if (typeof loader.decode === "function") await loader.decode();
      } catch (_error) {
        // The load event already confirms the image is usable on older browsers.
      }
      resolve(url);
    };
    loader.onerror = () => reject(new Error(`image load failed: ${url}`));
    loader.src = url;
  });
}

async function updateCameraImages(frame) {
  const generation = state.cameraLoadGeneration + 1;
  state.cameraLoadGeneration = generation;
  const views = Array.from(elements.cameraGrid.querySelectorAll(".camera-view"));
  const pending = views.map(async (view) => {
    const camera = view.dataset.camera;
    if (!frame.cameras[camera]) return { view, url: null, failed: false };
    const url = imageUrl(frame.index, camera);
    try {
      await loadDecodedImage(url);
      return { view, url, failed: false };
    } catch (_error) {
      return { view, url, failed: true };
    }
  });
  const loaded = await Promise.all(pending);
  if (generation !== state.cameraLoadGeneration) return;

  loaded.forEach(({ view, url, failed }) => {
    const image = view.querySelector("img");
    const unavailable = view.querySelector(".camera-unavailable");
    if (!url || failed) {
      image.classList.add("hidden");
      unavailable.classList.remove("hidden");
      unavailable.textContent = failed ? "图像读取失败" : "当前帧无图像";
      return;
    }
    image.src = url;
    image.dataset.frame = String(frame.index);
    image.classList.remove("hidden");
    unavailable.classList.add("hidden");
  });
}

function preloadNextFrame(frameIndex) {
  const next = Math.min(frameIndex + 1, state.detail.frames.length - 1);
  state.detail.cameraNames.forEach((camera) => {
    const image = new Image();
    image.src = imageUrl(next, camera.name);
  });
}

function renderFrame() {
  if (!state.detail?.frames.length) return;
  const frame = state.detail.frames[state.currentFrame];
  elements.frameSlider.value = String(state.currentFrame);
  elements.frameNumber.value = String(state.currentFrame + 1);
  elements.phasePlayhead.style.left = `${(state.currentFrame / Math.max(1, state.detail.frames.length - 1)) * 100}%`;
  elements.currentPhaseLabel.textContent = currentPhase(state.currentFrame);
  elements.currentSkill.textContent = frame.skillId;
  elements.currentSkill.className = `skill-chip ${segmentClass(frame.skillId)}`;
  elements.currentPromptVersion.textContent = `Prompt ${frame.promptVersion}`;
  elements.currentFrameStatus.textContent = frame.repaired
    ? `已修复 · ${frame.repairSources.join(", ")}`
    : frame.valid
      ? "原始对齐帧"
      : "无效帧";
  elements.currentPrompt.textContent = frame.task;
  elements.promptPanel.className = `prompt-panel ${segmentClass(frame.skillId)}`;
  elements.frameTimestamp.textContent = formatFrameTime(frame);
  updateCameraImages(frame);
  drawActivityChart();
  preloadNextFrame(state.currentFrame);
}

function setFrame(index) {
  if (!state.detail?.frames.length) return;
  state.currentFrame = Math.max(0, Math.min(Number(index) || 0, state.detail.frames.length - 1));
  renderFrame();
}

function percentile(values, fraction) {
  if (!values.length) return 1;
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.min(sorted.length - 1, Math.floor((sorted.length - 1) * fraction))] || 1;
}

function drawActivityChart() {
  if (!state.detail) return;
  const canvas = elements.activityChart;
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext("2d");
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  const cssWidth = rect.width;
  const cssHeight = rect.height;
  context.clearRect(0, 0, cssWidth, cssHeight);
  const frames = state.detail.frames;
  const count = Math.max(1, frames.length - 1);
  const colors = { s1: "#e4f1ed", s2: "#f2e7ee", s3: "#e3edf4", other: "#e6eef4" };
  state.detail.segments.forEach((segment) => {
    const left = (segment.startFrame / count) * cssWidth;
    const right = ((segment.endFrame + 1) / Math.max(1, frames.length)) * cssWidth;
    context.fillStyle = colors[segmentClass(segment.skillId)];
    context.fillRect(left, 0, Math.max(1, right - left), cssHeight);
  });
  context.strokeStyle = "#d6dde1";
  context.lineWidth = 1;
  for (let row = 1; row < 4; row += 1) {
    const y = (row / 4) * cssHeight;
    context.beginPath();
    context.moveTo(0, y + 0.5);
    context.lineTo(cssWidth, y + 0.5);
    context.stroke();
  }
  const armScale = percentile(frames.map((frame) => frame.leftArmStep), 0.96);
  const baseScale = percentile(frames.map((frame) => frame.baseSpeed), 0.96);
  const drawLine = (key, scale, color) => {
    context.beginPath();
    frames.forEach((frame, index) => {
      const x = (index / count) * cssWidth;
      const normalized = Math.min(1, frame[key] / Math.max(scale, 1e-6));
      const y = cssHeight - 8 - normalized * (cssHeight - 16);
      if (index === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.strokeStyle = color;
    context.lineWidth = 1.6;
    context.stroke();
  };
  drawLine("leftArmStep", armScale, "#16735f");
  drawLine("baseSpeed", baseScale, "#bc6a1c");
  const currentX = (state.currentFrame / count) * cssWidth;
  context.beginPath();
  context.moveTo(currentX, 0);
  context.lineTo(currentX, cssHeight);
  context.strokeStyle = "#172127";
  context.lineWidth = 1;
  context.stroke();
}

function stopPlayback() {
  state.playing = false;
  window.clearInterval(state.playTimer);
  state.playTimer = null;
  elements.playPause.innerHTML = '<i data-lucide="play"></i>';
  elements.playPause.title = "播放";
  elements.playPause.setAttribute("aria-label", "播放");
  createIcons();
}

function togglePlayback() {
  if (!state.detail) return;
  if (state.playing) {
    stopPlayback();
    return;
  }
  if (state.currentFrame >= state.detail.frames.length - 1) setFrame(0);
  state.playing = true;
  elements.playPause.innerHTML = '<i data-lucide="pause"></i>';
  elements.playPause.title = "暂停";
  elements.playPause.setAttribute("aria-label", "暂停");
  createIcons();
  const previewFps = 5;
  state.playTimer = window.setInterval(() => {
    const advance = Math.max(1, Math.round((state.detail.summary.fps * state.speed) / previewFps));
    if (state.currentFrame + advance >= state.detail.frames.length) {
      setFrame(state.detail.frames.length - 1);
      stopPlayback();
    } else {
      setFrame(state.currentFrame + advance);
    }
  }, 1000 / previewFps);
}

async function selectEpisode(episode) {
  stopPlayback();
  state.selectedKey = episodeKey(episode);
  state.currentFrame = 0;
  renderEpisodeList();
  elements.detailView.classList.add("hidden");
  elements.emptyState.classList.remove("hidden");
  elements.emptyMessage.textContent = `正在加载 ${episode.name}`;
  if (state.requestController) state.requestController.abort();
  state.requestController = new AbortController();
  try {
    const query = new URLSearchParams({ day: episode.day, episode: episode.name });
    const response = await fetch(`/api/episode?${query}`, { signal: state.requestController.signal });
    if (!response.ok) throw new Error((await response.json()).error || `HTTP ${response.status}`);
    state.detail = await response.json();
    history.replaceState(null, "", `#${encodeURIComponent(episode.day)}/${encodeURIComponent(episode.name)}`);
    renderHeading();
    renderPhaseTrack();
    renderCameraGrid();
    renderSegmentTable();
    elements.frameSlider.max = String(Math.max(0, state.detail.frames.length - 1));
    elements.frameTotal.textContent = `/ ${state.detail.frames.length}`;
    elements.frameNumber.max = String(state.detail.frames.length);
    elements.emptyState.classList.add("hidden");
    elements.detailView.classList.remove("hidden");
    setFrame(0);
    closeSidebar();
    createIcons();
  } catch (error) {
    if (error.name === "AbortError") return;
    elements.emptyMessage.textContent = `加载失败：${error.message}`;
    showToast(`Episode 加载失败：${error.message}`);
  }
}

function episodeFromHash() {
  const raw = location.hash.slice(1);
  if (!raw.includes("/")) return null;
  const [day, ...parts] = raw.split("/");
  const episode = decodeURIComponent(parts.join("/"));
  const decodedDay = decodeURIComponent(day);
  return state.episodes.find((item) => item.day === decodedDay && item.name === episode) || null;
}

async function loadIndex() {
  try {
    const response = await fetch("/api/index", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    state.index = await response.json();
    state.episodes = flattenEpisodes(state.index);
    elements.datasetRoot.textContent = state.index.root;
    setServiceStatus("online", `${state.index.episodeCount} 条数据`);
    renderDayFilter();
    applyFilters();
    const target = episodeFromHash() || state.filteredEpisodes[0] || state.episodes[0];
    if (target) selectEpisode(target);
    else elements.emptyMessage.textContent = "没有可显示的 frames.jsonl";
  } catch (error) {
    setServiceStatus("error", "连接失败");
    elements.emptyMessage.textContent = `无法读取数据目录：${error.message}`;
  }
}

function openSidebar() {
  elements.sidebar.classList.add("open");
  elements.sidebarBackdrop.classList.add("open");
}

function closeSidebar() {
  elements.sidebar.classList.remove("open");
  elements.sidebarBackdrop.classList.remove("open");
}

document.querySelectorAll(".mode-button").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".mode-button").forEach((item) => item.classList.remove("active"));
    button.classList.add("active");
    state.mode = button.dataset.mode;
    applyFilters();
    if (
      state.filteredEpisodes.length &&
      !state.filteredEpisodes.some((episode) => episodeKey(episode) === state.selectedKey)
    ) {
      selectEpisode(state.filteredEpisodes[0]);
    }
  });
});

document.querySelectorAll(".speed-switch button").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".speed-switch button").forEach((item) => item.classList.remove("active"));
    button.classList.add("active");
    state.speed = Number(button.dataset.speed);
  });
});

elements.episodeSearch.addEventListener("input", () => {
  state.search = elements.episodeSearch.value;
  applyFilters();
});
elements.dayFilter.addEventListener("change", () => {
  state.day = elements.dayFilter.value;
  applyFilters();
  if (
    state.filteredEpisodes.length &&
    !state.filteredEpisodes.some((episode) => episodeKey(episode) === state.selectedKey)
  ) {
    selectEpisode(state.filteredEpisodes[0]);
  }
});
elements.previousFrame.addEventListener("click", () => setFrame(state.currentFrame - 1));
elements.nextFrame.addEventListener("click", () => setFrame(state.currentFrame + 1));
elements.playPause.addEventListener("click", togglePlayback);
elements.frameSlider.addEventListener("input", () => {
  stopPlayback();
  setFrame(Number(elements.frameSlider.value));
});
elements.frameNumber.addEventListener("input", () => {
  stopPlayback();
  setFrame(Number(elements.frameNumber.value) - 1);
});
elements.phaseTrack.addEventListener("click", (event) => {
  if (!state.detail) return;
  const rect = elements.phaseTrack.getBoundingClientRect();
  const fraction = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
  setFrame(Math.round(fraction * (state.detail.frames.length - 1)));
});
elements.phaseTrack.addEventListener("keydown", (event) => {
  if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
    event.preventDefault();
    setFrame(state.currentFrame + (event.key === "ArrowLeft" ? -1 : 1));
  }
});
elements.activityChart.addEventListener("click", (event) => {
  if (!state.detail) return;
  const rect = elements.activityChart.getBoundingClientRect();
  const fraction = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
  setFrame(Math.round(fraction * (state.detail.frames.length - 1)));
});
elements.sidebarToggle.addEventListener("click", openSidebar);
elements.sidebarBackdrop.addEventListener("click", closeSidebar);
window.addEventListener("resize", drawActivityChart);
window.addEventListener("keydown", (event) => {
  if (event.target.matches("input, select, button")) return;
  if (event.key === "ArrowLeft") setFrame(state.currentFrame - 1);
  if (event.key === "ArrowRight") setFrame(state.currentFrame + 1);
  if (event.key === " ") {
    event.preventDefault();
    togglePlayback();
  }
});

createIcons();
loadIndex();
