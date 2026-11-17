/**
 * BUBIAI.GPS Telemetrija PWA v3.0
 * BAZINIS ir EKSPERTO režimai + Pilnas PWA diegimas
 * https://gps.bubiai.eu
 */

(() => {
  'use strict';

  const STORAGE_KEY = 'bubiai_gps_session_v3';
  const MODE_STORAGE_KEY = 'bubiai_gps_ui_mode';
  const CALIB_STORAGE_KEY = 'bubiai_gps_calib_v1';

  const EARTH_RADIUS_M = 6371000;
  const MOVING_THRESHOLD_KMH = 1.5;
  const MAX_USABLE_ACCURACY_M = 150;
  const COARSE_LOCATION_THRESHOLD_M = 1000;

  const OSM_TILE_URL = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
  const CARTO_DARK_URL = 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png';
  const DEFAULT_CENTER = [55.8569, 23.1364];

  let deferredInstallPrompt = null;

  const state = {
    uiMode: 'basic', // 'basic' | 'expert'
    isRunning: false,
    isPaused: false,
    isSimulating: false,
    watchId: null,
    simTimerId: null,
    clockTimerId: null,

    // Timers (seconds)
    totalElapsedSec: 0,
    movingElapsedSec: 0,
    lastTickTimestamp: null,

    // Speed metrics
    currentSpeedMs: 0,
    currentSpeedKmh: 0,
    currentSpeedKnots: 0,
    rawSpeedKmh: 0,
    rawDopplerMs: null,
    rawHaversineKmh: 0,
    maxSpeedKmh: 0,
    speedWindowMs: [],

    // Distance
    totalDistanceM: 0,

    // GPS Packet timing & rate
    lastFix: null,
    lastPacketWallMs: null,
    lastDeltaMs: null,
    gpsHz: 0,
    gpsPacketsCount: 0,

    // Altitude & Terrain
    currentAltitude: null,
    referenceAltitude: null,
    elevationGainM: 0,
    elevationLossM: 0,
    gradientPct: 0,
    gradientDeg: 0,
    verticalSpeedMmin: 0,
    verticalSpeedMs: 0,
    vamMh: 0,
    lastAltSample: null,

    // Heading / Compass
    currentHeadingDeg: null,

    // Accuracy & Coordinates
    currentLat: null,
    currentLon: null,
    horizAccuracyM: null,
    vertAccuracyM: null,

    // 3D Orientation & Inclinometer
    rawPitch: 0,
    rawRoll: 0,
    rawYaw: 0,
    pitchOffset: 0,
    rollOffset: 0,
    calibPitch: 0,
    calibRoll: 0,

    // IMU (Accelerometer & Gyroscope)
    gForceCur: 1.0,
    gForceMax: 1.0,
    linearAccelMs2: 0.0,
    linAccX: 0,
    linAccY: 0,
    linAccZ: 0,
    accelX: 0,
    accelY: 0,
    accelZ: 0,
    gyroAlpha: 0,
    gyroBeta: 0,
    gyroGamma: 0,
    imuIntervalMs: null,

    // Battery
    batteryStartPct: null,
    batteryCurPct: null,
    batteryCharging: false,
    batteryRemainingSec: null,

    // FPS
    fps: 60,

    // Wake Lock
    wakeLockEnabled: true,
    wakeLockSentinel: null,

    // Map preferences
    autoCenterMap: true,
    darkMapLayer: true,

    // Diagnostic log counter
    diagCount: 0,

    // Recorded track points
    points: []
  };

  // DOM References
  const el = {
    sessionStatusText: document.getElementById('sessionStatusText'),
    btnInstallApp: document.getElementById('btnInstallApp'),
    installModal: document.getElementById('installModal'),
    installModalClose: document.getElementById('installModalClose'),
    installModalOk: document.getElementById('installModalOk'),

    btnModeBasic: document.getElementById('btnModeBasic'),
    btnModeExpert: document.getElementById('btnModeExpert'),

    gpsSignalDot: document.getElementById('gpsSignalDot'),
    gpsSignalBars: document.getElementById('gpsSignalBars'),
    gpsQualityLabel: document.getElementById('gpsQualityLabel'),
    accHoriz: document.getElementById('accHoriz'),
    accVert: document.getElementById('accVert'),
    wakeLockBtn: document.getElementById('wakeLockBtn'),
    wakeLockText: document.getElementById('wakeLockText'),
    contrastBtn: document.getElementById('contrastBtn'),

    alertBanner: document.getElementById('alertBanner'),
    alertBannerText: document.getElementById('alertBannerText'),
    alertBannerClose: document.getElementById('alertBannerClose'),

    btnStart: document.getElementById('btnStart'),
    btnStartText: document.getElementById('btnStartText'),
    btnPause: document.getElementById('btnPause'),
    btnReset: document.getElementById('btnReset'),
    btnSim: document.getElementById('btnSim'),
    btnSimText: document.getElementById('btnSimText'),

    // Speedometer
    speedSourceBadge: document.getElementById('speedSourceBadge'),
    speedKmh: document.getElementById('speedKmh'),
    speedMs: document.getElementById('speedMs'),
    speedKnots: document.getElementById('speedKnots'),
    rawSpeedKmh: document.getElementById('rawSpeedKmh'),
    rawGpsSpeedMs: document.getElementById('rawGpsSpeedMs'),
    rawHaversineKmh: document.getElementById('rawHaversineKmh'),
    speedBarFill: document.getElementById('speedBarFill'),
    maxSpeedKmh: document.getElementById('maxSpeedKmh'),
    avgMovingSpeedKmh: document.getElementById('avgMovingSpeedKmh'),
    avgTotalSpeedKmh: document.getElementById('avgTotalSpeedKmh'),

    // Summary 4 Cards
    movingStateBadge: document.getElementById('movingStateBadge'),
    distancePrimary: document.getElementById('distancePrimary'),
    distanceUnit: document.getElementById('distanceUnit'),
    distanceMetersSub: document.getElementById('distanceMetersSub'),
    timerMoving: document.getElementById('timerMoving'),
    timerTotal: document.getElementById('timerTotal'),
    altitudeCur: document.getElementById('altitudeCur'),
    elevGain: document.getElementById('elevGain'),
    elevLoss: document.getElementById('elevLoss'),
    compassArrow: document.getElementById('compassArrow'),
    headingCardinal: document.getElementById('headingCardinal'),
    headingDeg: document.getElementById('headingDeg'),
    headingSubNote: document.getElementById('headingSubNote'),

    // Map
    mapContainerWrap: document.getElementById('mapContainerWrap'),
    btnCenterMap: document.getElementById('btnCenterMap'),
    btnToggleMapTheme: document.getElementById('btnToggleMapTheme'),
    mapPointsBadge: document.getElementById('mapPointsBadge'),
    mapLastUpdate: document.getElementById('mapLastUpdate'),

    // E1. Extended GPS & Geodesy
    gpsRateBadge: document.getElementById('gpsRateBadge'),
    coordLat: document.getElementById('coordLat'),
    coordLon: document.getElementById('coordLon'),
    coordDms: document.getElementById('coordDms'),
    btnCopyCoords: document.getElementById('btnCopyCoords'),
    copyCoordsText: document.getElementById('copyCoordsText'),
    expHorizAcc: document.getElementById('expHorizAcc'),
    expVertAcc: document.getElementById('expVertAcc'),
    gradientPct: document.getElementById('gradientPct'),
    gradientDeg: document.getElementById('gradientDeg'),
    vertSpeedMmin: document.getElementById('vertSpeedMmin'),
    vamReadout: document.getElementById('vamReadout'),
    vertSpeedMs: document.getElementById('vertSpeedMs'),
    gpsHzVal: document.getElementById('gpsHzVal'),
    gpsDeltaMs: document.getElementById('gpsDeltaMs'),
    gpsPacketsCount: document.getElementById('gpsPacketsCount'),
    altFilterBadge: document.getElementById('altFilterBadge'),

    // E2. 3D Orientation & Inclinometer
    btnCalibratePitch: document.getElementById('btnCalibratePitch'),
    btnResetCalib: document.getElementById('btnResetCalib'),
    horizonDisc: document.getElementById('horizonDisc'),
    orientPitch: document.getElementById('orientPitch'),
    orientRoll: document.getElementById('orientRoll'),
    orientYaw: document.getElementById('orientYaw'),
    calibOffsetText: document.getElementById('calibOffsetText'),

    // E3. IMU Sensors
    btnEnableMotion: document.getElementById('btnEnableMotion'),
    gForceDot: document.getElementById('gForceDot'),
    gForceCur: document.getElementById('gForceCur'),
    gForceMax: document.getElementById('gForceMax'),
    accelMs2: document.getElementById('accelMs2'),
    linAccX: document.getElementById('linAccX'),
    linAccY: document.getElementById('linAccY'),
    linAccZ: document.getElementById('linAccZ'),
    accelX: document.getElementById('accelX'),
    accelY: document.getElementById('accelY'),
    accelZ: document.getElementById('accelZ'),
    gyroAlpha: document.getElementById('gyroAlpha'),
    gyroBeta: document.getElementById('gyroBeta'),
    gyroGamma: document.getElementById('gyroGamma'),
    imuIntervalMs: document.getElementById('imuIntervalMs'),

    // E4-E6. Battery, Network, Hardware
    fpsBadge: document.getElementById('fpsBadge'),
    batLevel: document.getElementById('batLevel'),
    batState: document.getElementById('batState'),
    batDeltaNote: document.getElementById('batDeltaNote'),
    netType: document.getElementById('netType'),
    netDownlink: document.getElementById('netDownlink'),
    netRttNote: document.getElementById('netRttNote'),
    hwCoresRam: document.getElementById('hwCoresRam'),
    hwScreenNote: document.getElementById('hwScreenNote'),
    diagWakeLock: document.getElementById('diagWakeLock'),
    diagScreenAngle: document.getElementById('diagScreenAngle'),
    diagJsHeap: document.getElementById('diagJsHeap'),
    pointCount: document.getElementById('pointCount'),

    // E7. Chart, Export & Raw Log
    telemetryChart: document.getElementById('telemetryChart'),
    storageStatusText: document.getElementById('storageStatusText'),
    btnExportGpx: document.getElementById('btnExportGpx'),
    btnExportJson: document.getElementById('btnExportJson'),
    btnExportCsv: document.getElementById('btnExportCsv'),
    diagCountBadge: document.getElementById('diagCountBadge'),
    diagLogList: document.getElementById('diagLogList')
  };

  let map = null;
  let tileLayerDark = null;
  let tileLayerLight = null;
  let routePolyline = null;
  let currentMarker = null;
  let accuracyCircle = null;

  // =========================================================================
  // MATHEMATICAL & GEODETIC HELPERS
  // =========================================================================

  function toRad(deg) {
    return (deg * Math.PI) / 180;
  }

  function toDeg(rad) {
    return (rad * 180) / Math.PI;
  }

  function haversineDistanceM(lat1, lon1, lat2, lon2) {
    const dLat = toRad(lat2 - lat1);
    const dLon = toRad(lon2 - lon1);
    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) *
      Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return EARTH_RADIUS_M * c;
  }

  function computeBearingDeg(lat1, lon1, lat2, lon2) {
    const φ1 = toRad(lat1);
    const φ2 = toRad(lat2);
    const Δλ = toRad(lon2 - lon1);
    const y = Math.sin(Δλ) * Math.cos(φ2);
    const x =
      Math.cos(φ1) * Math.sin(φ2) -
      Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
    const θ = Math.atan2(y, x);
    return (toDeg(θ) + 360) % 360;
  }

  function degreesToCardinalLT(deg) {
    if (deg === null || deg === undefined || Number.isNaN(deg)) return '--';
    const dirs = ['Š', 'ŠR', 'R', 'PR', 'P', 'PV', 'V', 'ŠV'];
    const idx = Math.round(((deg % 360) / 45)) % 8;
    return dirs[idx];
  }

  /**
   * Convert decimal degrees to DMS string (e.g. 55°51'24.8"N)
   */
  function decimalToDMS(deg, isLat) {
    if (deg === null || deg === undefined || Number.isNaN(deg)) return '--°--\'--.-"';
    const dir = isLat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
    const abs = Math.abs(deg);
    const d = Math.floor(abs);
    const minFloat = (abs - d) * 60;
    const m = Math.floor(minFloat);
    const s = ((minFloat - m) * 60).toFixed(1);
    return `${String(d).padStart(2, '0')}°${String(m).padStart(2, '0')}'${String(s).padStart(4, '0')}"${dir}`;
  }

  function formatDuration(totalSec) {
    const sec = Math.max(0, Math.floor(totalSec));
    const h = Math.floor(sec / 3600);
    const m = Math.floor((sec % 3600) / 60);
    const s = sec % 60;
    return [h, m, s].map(v => String(v).padStart(2, '0')).join(':');
  }

  function showAlert(msg) {
    el.alertBannerText.textContent = msg;
    el.alertBanner.classList.remove('hidden');
  }

  function hideAlert() {
    el.alertBanner.classList.add('hidden');
  }

  /**
   * Keep a rolling list of the last 15 GPS fixes in the Live Raw Log
   */
  function addDiagLog(text, type = 'normal') {
    if (!el.diagLogList) return;
    const emptyEl = el.diagLogList.querySelector('.diag-empty');
    if (emptyEl) emptyEl.remove();

    state.diagCount += 1;
    if (el.diagCountBadge) {
      el.diagCountBadge.textContent = `Įrašai: ${state.diagCount}`;
    }

    const row = document.createElement('div');
    row.className = `diag-item ${type}`;
    row.textContent = text;
    el.diagLogList.prepend(row);

    while (el.diagLogList.children.length > 15) {
      el.diagLogList.removeChild(el.diagLogList.lastChild);
    }
  }

  // =========================================================================
  // 1. PWA INSTALL PROMPT & STANDALONE DETECTION
  // =========================================================================

  function initPwaInstall() {
    const isStandalone =
      window.matchMedia('(display-mode: standalone)').matches ||
      window.navigator.standalone === true;

    if (isStandalone && el.btnInstallApp) {
      el.btnInstallApp.classList.add('hidden');
    }

    window.addEventListener('beforeinstallprompt', e => {
      e.preventDefault();
      deferredInstallPrompt = e;
      if (el.btnInstallApp && !isStandalone) {
        el.btnInstallApp.classList.remove('hidden');
      }
    });

    window.addEventListener('appinstalled', () => {
      deferredInstallPrompt = null;
      if (el.btnInstallApp) {
        el.btnInstallApp.classList.add('hidden');
      }
    });

    if (el.btnInstallApp) {
      el.btnInstallApp.addEventListener('click', async () => {
        if (deferredInstallPrompt) {
          deferredInstallPrompt.prompt();
          try {
            const choice = await deferredInstallPrompt.userChoice;
            if (choice && choice.outcome === 'accepted') {
              el.btnInstallApp.classList.add('hidden');
            }
          } catch (_) {}
          deferredInstallPrompt = null;
        } else {
          if (el.installModal) {
            el.installModal.classList.remove('hidden');
          }
        }
      });
    }

    const closeModal = () => {
      if (el.installModal) el.installModal.classList.add('hidden');
    };
    if (el.installModalClose) el.installModalClose.addEventListener('click', closeModal);
    if (el.installModalOk) el.installModalOk.addEventListener('click', closeModal);
    if (el.installModal) {
      el.installModal.addEventListener('click', e => {
        if (e.target === el.installModal) closeModal();
      });
    }
  }

  // =========================================================================
  // 2. UI MODE SWITCHER (BAZINĖ vs EKSPERTO)
  // =========================================================================

  function setUiMode(mode) {
    const validMode = mode === 'expert' ? 'expert' : 'basic';
    state.uiMode = validMode;
    document.body.setAttribute('data-mode', validMode);

    if (el.btnModeBasic && el.btnModeExpert) {
      el.btnModeBasic.classList.toggle('active', validMode === 'basic');
      el.btnModeExpert.classList.toggle('active', validMode === 'expert');
    }

    try {
      localStorage.setItem(MODE_STORAGE_KEY, validMode);
    } catch (_) {}

    setTimeout(() => {
      if (map) map.invalidateSize();
      renderChart();
    }, 60);
  }

  function initUiMode() {
    let savedMode = 'basic';
    try {
      savedMode = localStorage.getItem(MODE_STORAGE_KEY) || 'basic';
    } catch (_) {}
    setUiMode(savedMode);

    if (el.btnModeBasic) {
      el.btnModeBasic.addEventListener('click', () => setUiMode('basic'));
    }
    if (el.btnModeExpert) {
      el.btnModeExpert.addEventListener('click', () => setUiMode('expert'));
    }
  }

  // =========================================================================
  // MAP INITIALIZATION (LEAFLET + OPENSTREETMAP)
  // =========================================================================

  function initMap() {
    if (typeof L === 'undefined') return;

    map = L.map('map', {
      center: DEFAULT_CENTER,
      zoom: 14,
      zoomControl: false,
      attributionControl: true
    });

    L.control.zoom({ position: 'topright' }).addTo(map);

    tileLayerLight = L.tileLayer(OSM_TILE_URL, {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
      maxZoom: 19
    });

    tileLayerDark = L.tileLayer(OSM_TILE_URL, {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> (Dark)',
      subdomains: 'abcd',
      maxZoom: 20
    });
    tileLayerDark.cartoReferenceUrl = CARTO_DARK_URL;

    tileLayerDark.addTo(map);
    if (el.mapContainerWrap) {
      el.mapContainerWrap.classList.add('map-dark-active');
    }

    routePolyline = L.polyline([], {
      color: '#00ffaa',
      weight: 4,
      opacity: 0.9,
      lineJoin: 'round'
    }).addTo(map);

    accuracyCircle = L.circle(DEFAULT_CENTER, {
      radius: 0,
      color: '#00ffaa',
      fillColor: '#00ffaa',
      fillOpacity: 0.12,
      weight: 1
    });

    const pulseIcon = L.divIcon({
      className: 'gps-map-marker',
      html: `<div style="width:14px;height:14px;border-radius:50%;background:#00ffaa;border:2px solid #000;box-shadow:0 0 12px #00ffaa;"></div>`,
      iconSize: [14, 14],
      iconAnchor: [7, 7]
    });

    currentMarker = L.marker(DEFAULT_CENTER, { icon: pulseIcon });

    map.on('dragstart', () => {
      state.autoCenterMap = false;
      el.btnCenterMap.classList.remove('active');
    });
  }

  function updateMapWithPoint(lat, lon, accuracyM) {
    if (!map) return;
    const latLng = [lat, lon];

    if (!map.hasLayer(currentMarker)) {
      currentMarker.addTo(map);
      accuracyCircle.addTo(map);
    }

    currentMarker.setLatLng(latLng);
    accuracyCircle.setLatLng(latLng);
    accuracyCircle.setRadius(Math.min(accuracyM || 10, 500));

    const pathCoords = state.points.map(p => [p.lat, p.lon]);
    routePolyline.setLatLngs(pathCoords);

    if (state.autoCenterMap) {
      map.panTo(latLng, { animate: true });
    }
  }

  // =========================================================================
  // REAL-TIME SPEED & ELEVATION PROFILE CANVAS CHART
  // =========================================================================

  function renderChart() {
    const canvas = el.telemetryChart;
    if (!canvas || state.uiMode !== 'expert') return;
    const ctx = canvas.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;

    if (canvas.width !== Math.floor(rect.width * dpr) || canvas.height !== Math.floor(rect.height * dpr)) {
      canvas.width = Math.floor(rect.width * dpr);
      canvas.height = Math.floor(rect.height * dpr);
    }

    ctx.save();
    ctx.scale(dpr, dpr);

    const w = rect.width;
    const h = rect.height;
    const pad = { top: 14, right: 38, bottom: 20, left: 36 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = '#162234';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 3; i++) {
      const y = pad.top + (plotH / 3) * i;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(w - pad.right, y);
      ctx.stroke();
    }

    const recentPoints = state.points.slice(-150);
    if (recentPoints.length < 2) {
      ctx.fillStyle = '#64748b';
      ctx.font = '600 11px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.fillText('Laukiama judėjimo telemetrijos duomenų...', w / 2, h / 2 + 4);
      ctx.restore();
      return;
    }

    const speeds = recentPoints.map(p => p.speedKmh || 0);
    const alts = recentPoints.map(p => (p.alt !== null && p.alt !== undefined ? p.alt : 0));

    const maxSpd = Math.max(20, ...speeds) * 1.1;
    const minAlt = Math.min(...alts) - 3;
    const maxAlt = Math.max(minAlt + 10, ...alts) + 3;

    const getX = idx => pad.left + (idx / (recentPoints.length - 1)) * plotW;
    const getYSpeed = s => pad.top + plotH - (Math.min(s, maxSpd) / maxSpd) * plotH;
    const getYAlt = a => pad.top + plotH - ((a - minAlt) / (maxAlt - minAlt)) * plotH;

    // 1. Altitude Area
    ctx.beginPath();
    recentPoints.forEach((p, i) => {
      const x = getX(i);
      const y = getYAlt(p.alt !== null && p.alt !== undefined ? p.alt : minAlt);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 1.8;
    ctx.stroke();

    ctx.lineTo(getX(recentPoints.length - 1), pad.top + plotH);
    ctx.lineTo(getX(0), pad.top + plotH);
    ctx.closePath();
    const gradAlt = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    gradAlt.addColorStop(0, 'rgba(16, 185, 129, 0.25)');
    gradAlt.addColorStop(1, 'rgba(16, 185, 129, 0.0)');
    ctx.fillStyle = gradAlt;
    ctx.fill();

    // 2. Speed Area
    ctx.beginPath();
    recentPoints.forEach((p, i) => {
      const x = getX(i);
      const y = getYSpeed(p.speedKmh || 0);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#00ffaa';
    ctx.lineWidth = 2.2;
    ctx.stroke();

    ctx.lineTo(getX(recentPoints.length - 1), pad.top + plotH);
    ctx.lineTo(getX(0), pad.top + plotH);
    ctx.closePath();
    const gradSpd = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    gradSpd.addColorStop(0, 'rgba(0, 255, 170, 0.28)');
    gradSpd.addColorStop(1, 'rgba(0, 255, 170, 0.0)');
    ctx.fillStyle = gradSpd;
    ctx.fill();

    ctx.font = '600 9px "JetBrains Mono", monospace';
    ctx.fillStyle = '#00ffaa';
    ctx.textAlign = 'right';
    ctx.fillText(`${Math.round(maxSpd)}`, pad.left - 5, pad.top + 8);
    ctx.fillText('0', pad.left - 5, pad.top + plotH);

    ctx.fillStyle = '#10b981';
    ctx.textAlign = 'left';
    ctx.fillText(`${Math.round(maxAlt)}m`, w - pad.right + 4, pad.top + 8);
    ctx.fillText(`${Math.round(minAlt)}m`, w - pad.right + 4, pad.top + plotH);

    ctx.restore();
  }

  // =========================================================================
  // GPS POSITION PROCESSING (WATCHPOSITION HANDLER)
  // =========================================================================

  function handlePosition(pos) {
    const coords = pos.coords;
    const nowMs = Date.now();
    const gpsTimeMs = pos.timestamp || nowMs;

    // Packet rate & delta interval
    state.gpsPacketsCount += 1;
    if (state.lastPacketWallMs !== null) {
      const dtMs = Math.max(1, nowMs - state.lastPacketWallMs);
      state.lastDeltaMs = dtMs;
      state.gpsHz = 1000 / dtMs;
    }
    state.lastPacketWallMs = nowMs;

    const lat = coords.latitude;
    const lon = coords.longitude;
    const accuracy = typeof coords.accuracy === 'number' ? coords.accuracy : 9999;
    const alt = typeof coords.altitude === 'number' ? coords.altitude : null;
    const altAcc = typeof coords.altitudeAccuracy === 'number' ? coords.altitudeAccuracy : null;
    const nativeSpeedMs =
      typeof coords.speed === 'number' && !Number.isNaN(coords.speed) && coords.speed >= 0
        ? coords.speed
        : null;

    state.currentLat = lat;
    state.currentLon = lon;
    state.horizAccuracyM = accuracy;
    state.vertAccuracyM = altAcc;
    state.rawDopplerMs = nativeSpeedMs;

    let deltaDistM = 0;
    let deltaTimeSec = 0;
    let haversineSpeedMs = 0;

    if (state.lastFix) {
      deltaDistM = haversineDistanceM(state.lastFix.lat, state.lastFix.lon, lat, lon);
      deltaTimeSec = Math.max(0.25, (nowMs - state.lastFix.timestamp) / 1000);
      haversineSpeedMs = deltaDistM / deltaTimeSec;
    }

    const haversineSpeedKmh = haversineSpeedMs * 3.6;
    const nativeSpeedKmh = nativeSpeedMs !== null ? nativeSpeedMs * 3.6 : null;

    state.rawHaversineKmh = haversineSpeedKmh;
    state.rawSpeedKmh = nativeSpeedKmh !== null ? nativeSpeedKmh : haversineSpeedKmh;

    if (el.rawSpeedKmh) el.rawSpeedKmh.textContent = `${state.rawSpeedKmh.toFixed(1)} km/h`;
    if (el.rawGpsSpeedMs) {
      el.rawGpsSpeedMs.textContent =
        nativeSpeedMs !== null ? `${nativeSpeedMs.toFixed(2)} m/s` : 'null';
    }
    if (el.rawHaversineKmh) {
      el.rawHaversineKmh.textContent = `${haversineSpeedKmh.toFixed(1)} km/h (Δd=${deltaDistM.toFixed(1)}m)`;
    }

    const timeStr = new Date(gpsTimeMs).toLocaleTimeString('lt-LT');
    const dtLabel = state.lastDeltaMs !== null ? `${state.lastDeltaMs}ms` : '--ms';

    // 1. COARSE LOCATION CHECK (coords.accuracy > 1000)
    if (accuracy > COARSE_LOCATION_THRESHOLD_M) {
      showAlert(
        `Gaunama tik apytikslė tinklo vieta (±${Math.round(accuracy)} m). Įjunkite „Tikslią vietovę“ (Precise Location) naršyklės leidimuose ir išeikite į atvirą vietą.`
      );
      addDiagLog(
        `[${timeStr}] ${lat.toFixed(5)}, ${lon.toFixed(5)} | Alt: ${alt !== null ? alt.toFixed(0) + 'm' : '--'} | ${state.rawSpeedKmh.toFixed(1)} km/h | ±${Math.round(accuracy)}m (APYTIKSLĖ) | Δt:${dtLabel}`,
        'coarse'
      );
      updateUI();
      updateMapWithPoint(lat, lon, accuracy);
      el.mapLastUpdate.textContent = `Atnaujinta: ${timeStr} (±${Math.round(accuracy)}m)`;
      return;
    } else if (el.alertBannerText.textContent.includes('apytikslė tinklo vieta')) {
      hideAlert();
    }

    // 2. USABLE FIXES (coords.accuracy <= 150m)
    const isUsableAccuracy = accuracy <= MAX_USABLE_ACCURACY_M;
    let effectiveSpeedMs = 0;

    if (isUsableAccuracy) {
      if (nativeSpeedMs !== null && nativeSpeedMs > 0) {
        effectiveSpeedMs = nativeSpeedMs;
        el.speedSourceBadge.textContent = state.isSimulating ? 'SIMULIACIJA' : 'GPS DOPPLER';
      } else if (state.lastFix && deltaDistM >= 0.5 && deltaTimeSec > 0) {
        state.speedWindowMs.push(haversineSpeedMs);
        if (state.speedWindowMs.length > 3) state.speedWindowMs.shift();
        effectiveSpeedMs = state.speedWindowMs.reduce((a, b) => a + b, 0) / state.speedWindowMs.length;
        el.speedSourceBadge.textContent = 'HAVERSINE Δd/Δt';
      } else {
        state.speedWindowMs = [];
        effectiveSpeedMs = 0;
        el.speedSourceBadge.textContent = nativeSpeedMs === 0 ? 'GPS (0.0)' : 'HAVERSINE Δd/Δt';
      }

      state.currentSpeedMs = effectiveSpeedMs;
      state.currentSpeedKmh = effectiveSpeedMs * 3.6;
      state.currentSpeedKnots = effectiveSpeedMs * 1.943844;

      // Heading / Compass
      if (typeof coords.heading === 'number' && !Number.isNaN(coords.heading) && coords.heading >= 0) {
        state.currentHeadingDeg = coords.heading;
      } else if (state.lastFix && deltaDistM >= 1.5) {
        state.currentHeadingDeg = computeBearingDeg(state.lastFix.lat, state.lastFix.lon, lat, lon);
      }

      // Altimeter & Terrain
      if (alt !== null) {
        state.currentAltitude = alt;
        const elevThreshold = Math.max(2.5, altAcc ? Math.min(8, altAcc * 0.45) : 2.8);
        if (el.altFilterBadge) {
          el.altFilterBadge.textContent = `Slenkstis: ±${elevThreshold.toFixed(1)}m`;
        }

        if (state.referenceAltitude === null) {
          state.referenceAltitude = alt;
        } else if (state.isRunning && !state.isPaused) {
          const diffAlt = alt - state.referenceAltitude;
          if (Math.abs(diffAlt) >= elevThreshold) {
            if (diffAlt > 0) state.elevationGainM += diffAlt;
            else state.elevationLossM += Math.abs(diffAlt);
            state.referenceAltitude = alt;
          }
        }

        if (state.lastAltSample) {
          const dtSec = (nowMs - state.lastAltSample.timeMs) / 1000;
          const dtMin = dtSec / 60;
          const dAlt = alt - state.lastAltSample.alt;
          const dHoriz = haversineDistanceM(state.lastAltSample.lat, state.lastAltSample.lon, lat, lon);

          if (dtMin > 0.01) {
            state.verticalSpeedMmin = state.verticalSpeedMmin * 0.6 + (dAlt / dtMin) * 0.4;
            state.verticalSpeedMs = state.verticalSpeedMmin / 60;
            state.vamMh = Math.max(0, state.verticalSpeedMmin * 60);
          }
          if (dHoriz >= 3.0) {
            const rawGrad = (dAlt / dHoriz) * 100;
            const clampedGrad = Math.max(-45, Math.min(45, rawGrad));
            state.gradientPct = state.gradientPct * 0.65 + clampedGrad * 0.35;
            state.gradientDeg = toDeg(Math.atan(state.gradientPct / 100));
          } else if (state.currentSpeedKmh < 1.0) {
            state.gradientPct = 0;
            state.gradientDeg = 0;
            state.verticalSpeedMmin = 0;
            state.verticalSpeedMs = 0;
            state.vamMh = 0;
          }
        }

        if (!state.lastAltSample || nowMs - state.lastAltSample.timeMs >= 2000) {
          state.lastAltSample = { alt, lat, lon, timeMs: nowMs };
        }
      }

      // Active Trip Recording
      if (state.isRunning && !state.isPaused) {
        if (state.currentSpeedKmh > state.maxSpeedKmh) {
          state.maxSpeedKmh = state.currentSpeedKmh;
        }

        if (state.lastFix && deltaDistM >= 0.5 && (state.currentSpeedKmh >= 0.6 || deltaDistM >= 1.5)) {
          state.totalDistanceM += deltaDistM;
        }

        state.points.push({
          lat: Number(lat.toFixed(6)),
          lon: Number(lon.toFixed(6)),
          alt: alt !== null ? Number(alt.toFixed(1)) : null,
          speedKmh: Number(state.currentSpeedKmh.toFixed(2)),
          speedMs: Number(state.currentSpeedMs.toFixed(2)),
          speedKnots: Number(state.currentSpeedKnots.toFixed(2)),
          dopplerMs: nativeSpeedMs !== null ? Number(nativeSpeedMs.toFixed(2)) : null,
          haversineKmh: Number(haversineSpeedKmh.toFixed(2)),
          acc: Number(accuracy.toFixed(1)),
          altAcc: altAcc !== null ? Number(altAcc.toFixed(1)) : null,
          heading: state.currentHeadingDeg !== null ? Math.round(state.currentHeadingDeg) : null,
          distM: Number(state.totalDistanceM.toFixed(1)),
          gradPct: Number(state.gradientPct.toFixed(1)),
          vertMmin: Number(state.verticalSpeedMmin.toFixed(1)),
          gForce: Number(state.gForceCur.toFixed(2)),
          pitch: Number(state.calibPitch.toFixed(1)),
          roll: Number(state.calibRoll.toFixed(1)),
          yaw: Number(state.rawYaw.toFixed(1)),
          batPct: state.batteryCurPct,
          dtMs: state.lastDeltaMs || 0,
          timestamp: nowMs
        });
        saveSessionToStorage();
      }

      state.lastFix = { lat, lon, timestamp: nowMs, accuracy };
    }

    // Live Raw Log (Last 15 fixes)
    addDiagLog(
      `[${timeStr}] Lat:${lat.toFixed(5)} Lon:${lon.toFixed(5)} | Alt:${alt !== null ? alt.toFixed(0) + 'm' : '--'} | ${state.currentSpeedKmh.toFixed(1)} km/h | ±${Math.round(accuracy)}m | Δt:${dtLabel}`,
      isUsableAccuracy ? 'normal' : 'coarse'
    );

    updateUI();
    updateMapWithPoint(lat, lon, accuracy);
    renderChart();

    el.mapLastUpdate.textContent = `Atnaujinta: ${timeStr}`;
  }

  function handlePositionError(err) {
    const timeStr = new Date().toLocaleTimeString('lt-LT');

    if (err.code === 3) {
      el.gpsQualityLabel.textContent = 'Laukiama tikslaus GPS palydovų signalo...';
      if (state.isRunning && !state.isPaused) {
        el.sessionStatusText.textContent = 'Laukiama tikslaus GPS palydovų signalo...';
      }
      addDiagLog(`[${timeStr}] BŪSENA: Laukiama tikslaus GPS palydovų signalo... (watchPosition aktyvus)`, 'timeout');
      return;
    }

    if (err.code === 1) {
      showAlert('Geolokacijos leidimas atmestas. Suteikite naršyklei tikslios vietovės (Precise Location) leidimą.');
      el.gpsQualityLabel.textContent = 'LEIDIMAS ATMESTAS';
      addDiagLog(`[${timeStr}] KLAIDA: Geolokacijos leidimas atmestas (code 1)`, 'timeout');
    } else if (err.code === 2) {
      el.gpsQualityLabel.textContent = 'Laukiama tikslaus GPS palydovų signalo...';
      addDiagLog(`[${timeStr}] BŪSENA: Pozicija dar nepasiekiama (code 2) – tęsiama paieška`, 'timeout');
    } else {
      addDiagLog(`[${timeStr}] GPS pranešimas: ${err.message}`, 'timeout');
    }
  }

  // =========================================================================
  // UI RENDERING
  // =========================================================================

  function updateUI() {
    // 1. Speedometer
    el.speedKmh.textContent = state.currentSpeedKmh.toFixed(1);
    el.speedMs.textContent = state.currentSpeedMs.toFixed(2);
    if (el.speedKnots) el.speedKnots.textContent = state.currentSpeedKnots.toFixed(1);
    el.maxSpeedKmh.textContent = state.maxSpeedKmh.toFixed(1);

    const barPct = Math.min(100, (state.currentSpeedKmh / 140) * 100);
    el.speedBarFill.style.width = `${barPct}%`;

    const avgTotal =
      state.totalElapsedSec > 2
        ? (state.totalDistanceM / 1000) / (state.totalElapsedSec / 3600)
        : 0;
    const avgMoving =
      state.movingElapsedSec > 2
        ? (state.totalDistanceM / 1000) / (state.movingElapsedSec / 3600)
        : 0;

    if (el.avgTotalSpeedKmh) el.avgTotalSpeedKmh.textContent = avgTotal.toFixed(1);
    el.avgMovingSpeedKmh.textContent = avgMoving.toFixed(1);

    // 2. Summary 4 Cards (Distance, Moving Time, Altitude+Gain, Compass)
    if (state.totalDistanceM < 1000) {
      el.distancePrimary.textContent = Math.round(state.totalDistanceM).toString();
      el.distanceUnit.textContent = 'm';
      el.distanceMetersSub.textContent = `${(state.totalDistanceM / 1000).toFixed(3)} km`;
    } else {
      el.distancePrimary.textContent = (state.totalDistanceM / 1000).toFixed(2);
      el.distanceUnit.textContent = 'km';
      el.distanceMetersSub.textContent = `${Math.round(state.totalDistanceM)} m`;
    }

    el.timerTotal.textContent = formatDuration(state.totalElapsedSec);
    el.timerMoving.textContent = formatDuration(state.movingElapsedSec);

    const isMovingNow = state.currentSpeedKmh >= MOVING_THRESHOLD_KMH;
    el.movingStateBadge.textContent = isMovingNow ? 'JUDAMA' : 'STOVIMA';
    el.movingStateBadge.classList.toggle('moving', isMovingNow);

    el.altitudeCur.textContent =
      state.currentAltitude !== null ? state.currentAltitude.toFixed(1) : '--';
    el.elevGain.textContent = `+${Math.round(state.elevationGainM)} m`;
    el.elevLoss.textContent = `-${Math.round(state.elevationLossM)} m`;

    if (state.currentHeadingDeg !== null) {
      const deg = Math.round(state.currentHeadingDeg) % 360;
      const card = degreesToCardinalLT(deg);
      el.headingCardinal.textContent = card;
      el.headingDeg.textContent = `${deg}°`;
      if (el.headingSubNote) el.headingSubNote.textContent = `Kursas: ${card} ${deg}°`;
      el.compassArrow.style.transform = `rotate(${deg}deg)`;
    } else {
      el.headingCardinal.textContent = '--';
      el.headingDeg.textContent = '---°';
    }

    // 3. Signal Status Indicator (Silpnas / Geras / Puikus)
    if (state.horizAccuracyM !== null) {
      const hAcc = Math.round(state.horizAccuracyM);
      const vAcc = state.vertAccuracyM !== null ? `±${Math.round(state.vertAccuracyM)}m` : '--m';
      el.accHoriz.textContent = `H: ±${hAcc}m`;
      el.accVert.textContent = `V: ${vAcc}`;
      if (el.expHorizAcc) el.expHorizAcc.textContent = hAcc.toString();
      if (el.expVertAcc) {
        el.expVertAcc.textContent =
          state.vertAccuracyM !== null ? Math.round(state.vertAccuracyM).toString() : '--';
      }

      if (hAcc <= 12) {
        el.gpsQualityLabel.textContent = 'GPS: PUIKUS';
        el.gpsSignalDot.className = 'signal-dot good';
        el.gpsSignalBars.className = 'signal-bars bars-4';
      } else if (hAcc <= MAX_USABLE_ACCURACY_M) {
        el.gpsQualityLabel.textContent = 'GPS: GERAS';
        el.gpsSignalDot.className = 'signal-dot medium';
        el.gpsSignalBars.className = 'signal-bars bars-3';
      } else {
        el.gpsQualityLabel.textContent = 'GPS: SILPNAS';
        el.gpsSignalDot.className = 'signal-dot';
        el.gpsSignalBars.className = 'signal-bars bars-1';
      }
    }

    // 4. Expert E1: Extended Geodesy & Packet Rate
    if (state.currentLat !== null && state.currentLon !== null) {
      el.coordLat.textContent = `${state.currentLat.toFixed(6)}°`;
      el.coordLon.textContent = `${state.currentLon.toFixed(6)}°`;
      if (el.coordDms) {
        el.coordDms.textContent = `${decimalToDMS(state.currentLat, true)} ${decimalToDMS(state.currentLon, false)}`;
      }
    }

    if (el.gradientPct) {
      el.gradientPct.textContent = `${state.gradientPct >= 0 ? '+' : ''}${state.gradientPct.toFixed(1)}%`;
    }
    if (el.gradientDeg) {
      el.gradientDeg.textContent = `(${state.gradientDeg >= 0 ? '+' : ''}${state.gradientDeg.toFixed(1)}°)`;
    }
    if (el.vertSpeedMmin) {
      el.vertSpeedMmin.textContent = `${state.verticalSpeedMmin >= 0 ? '+' : ''}${state.verticalSpeedMmin.toFixed(1)}`;
    }
    if (el.vamReadout) {
      el.vamReadout.textContent = `m/min (VAM: ${Math.round(state.vamMh)} m/val.)`;
    }
    if (el.vertSpeedMs) {
      el.vertSpeedMs.textContent = `${state.verticalSpeedMs >= 0 ? '+' : ''}${state.verticalSpeedMs.toFixed(2)} m/s`;
    }
    if (el.gpsHzVal) {
      el.gpsHzVal.textContent = `${state.gpsHz.toFixed(2)} Hz`;
    }
    if (el.gpsDeltaMs) {
      el.gpsDeltaMs.textContent = state.lastDeltaMs !== null ? `${state.lastDeltaMs} ms` : '-- ms';
    }
    if (el.gpsRateBadge) {
      el.gpsRateBadge.textContent = `${state.gpsHz.toFixed(2)} Hz · Δt: ${state.lastDeltaMs !== null ? state.lastDeltaMs : '--'} ms`;
    }
    if (el.gpsPacketsCount) {
      el.gpsPacketsCount.textContent = state.gpsPacketsCount.toString();
    }

    // Points count
    if (el.pointCount) el.pointCount.textContent = state.points.length.toString();
    el.mapPointsBadge.textContent = `${state.points.length} taškų`;
  }

  // =========================================================================
  // TIMERS & TRIP CONTROLS (START / PAUSE / RESET / DEMO)
  // =========================================================================

  function startClockLoop() {
    if (state.clockTimerId) clearInterval(state.clockTimerId);
    state.lastTickTimestamp = Date.now();
    state.clockTimerId = setInterval(() => {
      updateSystemDiagnostics();
      if (!state.isRunning || state.isPaused) {
        state.lastTickTimestamp = Date.now();
        return;
      }
      const now = Date.now();
      const dtSec = (now - state.lastTickTimestamp) / 1000;
      state.lastTickTimestamp = now;

      state.totalElapsedSec += dtSec;
      if (state.currentSpeedKmh >= MOVING_THRESHOLD_KMH) {
        state.movingElapsedSec += dtSec;
      }
      updateUI();
    }, 500);
  }

  function startTracking() {
    if (state.batteryCurPct !== null && state.batteryStartPct === null) {
      state.batteryStartPct = state.batteryCurPct;
    }

    if (state.isRunning && state.isPaused) {
      state.isPaused = false;
      state.lastTickTimestamp = Date.now();
      el.sessionStatusText.textContent = state.isSimulating ? 'DEMO VYKDOMA' : 'FIKSUOJAMA (GPS)';
      el.btnStartText.textContent = 'FIKSUOJAMA';
      el.btnStart.classList.add('recording');
      el.btnPause.disabled = false;
      if (state.wakeLockEnabled) requestWakeLock();
      return;
    }

    if (state.isRunning) return;

    state.isRunning = true;
    state.isPaused = false;
    el.btnStartText.textContent = 'FIKSUOJAMA';
    el.btnStart.classList.add('recording');
    el.btnPause.disabled = false;
    el.sessionStatusText.textContent = state.isSimulating ? 'DEMO VYKDOMA' : 'FIKSUOJAMA (GPS)';

    if (state.wakeLockEnabled) {
      requestWakeLock();
    }

    startClockLoop();

    if (!state.isSimulating) {
      ensureLiveGeolocation();
    }
  }

  function ensureLiveGeolocation() {
    if (!('geolocation' in navigator)) {
      showAlert('Ši naršyklė nepalaiko Geolocation API.');
      return;
    }
    if (state.watchId !== null) {
      return;
    }
    state.watchId = navigator.geolocation.watchPosition(handlePosition, handlePositionError, {
      enableHighAccuracy: true,
      timeout: 30000,
      maximumAge: 0
    });
  }

  function pauseTracking() {
    if (!state.isRunning || state.isPaused) return;
    state.isPaused = true;
    state.currentSpeedKmh = 0;
    state.currentSpeedMs = 0;
    state.currentSpeedKnots = 0;
    el.sessionStatusText.textContent = 'PAUZĖ';
    el.btnStartText.textContent = 'TĘSTI';
    el.btnStart.classList.remove('recording');
    el.btnPause.disabled = true;
    saveSessionToStorage();
    updateUI();
  }

  function resetTracking() {
    stopSimulation();

    state.isRunning = false;
    state.isPaused = false;
    state.totalElapsedSec = 0;
    state.movingElapsedSec = 0;
    state.currentSpeedMs = 0;
    state.currentSpeedKmh = 0;
    state.currentSpeedKnots = 0;
    state.rawSpeedKmh = 0;
    state.rawHaversineKmh = 0;
    state.maxSpeedKmh = 0;
    state.speedWindowMs = [];
    state.totalDistanceM = 0;
    state.lastFix = null;
    state.referenceAltitude = state.currentAltitude;
    state.elevationGainM = 0;
    state.elevationLossM = 0;
    state.gradientPct = 0;
    state.gradientDeg = 0;
    state.verticalSpeedMmin = 0;
    state.verticalSpeedMs = 0;
    state.vamMh = 0;
    state.lastAltSample = null;
    state.gForceMax = 1.0;
    state.batteryStartPct = state.batteryCurPct;
    state.points = [];

    localStorage.removeItem(STORAGE_KEY);
    if (routePolyline) routePolyline.setLatLngs([]);

    el.sessionStatusText.textContent = 'ATSTATYTA // PASIRUOŠĘS';
    el.btnStartText.textContent = 'START';
    el.btnStart.classList.remove('recording');
    el.btnPause.disabled = true;
    if (el.storageStatusText) el.storageStatusText.textContent = 'Sesija išvalyta';

    updateUI();
    renderChart();
    ensureLiveGeolocation();
  }

  // =========================================================================
  // DEMO / SIMULATION MODE
  // =========================================================================

  let simAngle = 0;
  let simLat = DEFAULT_CENTER[0];
  let simLon = DEFAULT_CENTER[1];
  let simAlt = 118.0;

  function toggleSimulation() {
    if (state.isSimulating) {
      stopSimulation();
      ensureLiveGeolocation();
      return;
    }

    if (state.watchId !== null) {
      navigator.geolocation.clearWatch(state.watchId);
      state.watchId = null;
    }

    state.isSimulating = true;
    el.btnSim.classList.add('active');
    el.btnSimText.textContent = 'STOP DEMO';

    if (!state.isRunning || state.isPaused) {
      startTracking();
    }
    el.sessionStatusText.textContent = 'DEMO TELEMETRIJA';

    if (state.points.length > 0) {
      const last = state.points[state.points.length - 1];
      simLat = last.lat;
      simLon = last.lon;
      simAlt = last.alt || 118.0;
    }

    state.simTimerId = setInterval(() => {
      if (state.isPaused) return;
      simAngle += 0.08;

      const simSpeedMs = 11.5 + Math.sin(simAngle * 0.7) * 6.5 + Math.cos(simAngle * 1.9) * 2.5;
      const headingRad = simAngle * 0.35;
      const headingDeg = (toDeg(headingRad) + 360) % 360;

      const distStepM = simSpeedMs * 1.0;
      const dLat = (distStepM * Math.cos(headingRad)) / EARTH_RADIUS_M;
      const dLon = (distStepM * Math.sin(headingRad)) / (EARTH_RADIUS_M * Math.cos(toRad(simLat)));

      simLat += toDeg(dLat);
      simLon += toDeg(dLon);
      simAlt += Math.sin(simAngle * 0.5) * 1.4;

      const gx = Math.sin(simAngle * 1.3) * 1.8;
      const gy = Math.cos(simAngle * 0.9) * 1.4;
      updateImuMetrics(
        { x: gx * 0.6, y: gy * 0.6, z: Math.sin(simAngle) * 0.4 },
        { x: gx, y: gy, z: 9.81 + Math.sin(simAngle) * 0.6 },
        { alpha: Math.sin(simAngle) * 12, beta: Math.cos(simAngle) * 5, gamma: Math.sin(simAngle * 0.5) * 8 },
        16.7
      );
      updateOrientationMetrics(headingDeg, Math.sin(simAngle * 0.5) * 6.5, Math.cos(simAngle * 0.8) * 8.2);

      handlePosition({
        coords: {
          latitude: simLat,
          longitude: simLon,
          accuracy: 4.2,
          altitude: simAlt,
          altitudeAccuracy: 3.0,
          heading: headingDeg,
          speed: simSpeedMs
        },
        timestamp: Date.now()
      });
    }, 1000);
  }

  function stopSimulation() {
    if (state.simTimerId) {
      clearInterval(state.simTimerId);
      state.simTimerId = null;
    }
    state.isSimulating = false;
    el.btnSim.classList.remove('active');
    el.btnSimText.textContent = 'DEMO';
  }

  // =========================================================================
  // E2 & E3: 3D ORIENTATION, INCLINOMETER & IMU SENSORS
  // =========================================================================

  function loadCalibration() {
    try {
      const raw = localStorage.getItem(CALIB_STORAGE_KEY);
      if (raw) {
        const parsed = JSON.parse(raw);
        state.pitchOffset = Number(parsed.pitchOffset) || 0;
        state.rollOffset = Number(parsed.rollOffset) || 0;
      }
    } catch (_) {}
    updateCalibLabel();
  }

  function calibratePitchZero() {
    state.pitchOffset = state.rawPitch;
    state.rollOffset = state.rawRoll;
    try {
      localStorage.setItem(
        CALIB_STORAGE_KEY,
        JSON.stringify({ pitchOffset: state.pitchOffset, rollOffset: state.rollOffset })
      );
    } catch (_) {}
    updateOrientationMetrics(state.rawYaw, state.rawPitch, state.rawRoll);
    updateCalibLabel();
  }

  function resetPitchCalibration() {
    state.pitchOffset = 0;
    state.rollOffset = 0;
    localStorage.removeItem(CALIB_STORAGE_KEY);
    updateOrientationMetrics(state.rawYaw, state.rawPitch, state.rawRoll);
    updateCalibLabel();
  }

  function updateCalibLabel() {
    if (el.calibOffsetText) {
      el.calibOffsetText.textContent = `β₀=${state.pitchOffset.toFixed(1)}°, γ₀=${state.rollOffset.toFixed(1)}°`;
    }
  }

  function updateOrientationMetrics(alpha, beta, gamma) {
    state.rawYaw = typeof alpha === 'number' ? (alpha + 360) % 360 : 0;
    state.rawPitch = typeof beta === 'number' ? beta : 0;
    state.rawRoll = typeof gamma === 'number' ? gamma : 0;

    state.calibPitch = Math.max(-180, Math.min(180, state.rawPitch - state.pitchOffset));
    state.calibRoll = Math.max(-90, Math.min(90, state.rawRoll - state.rollOffset));

    if (el.orientPitch) el.orientPitch.textContent = state.calibPitch.toFixed(1);
    if (el.orientRoll) el.orientRoll.textContent = state.calibRoll.toFixed(1);
    if (el.orientYaw) el.orientYaw.textContent = state.rawYaw.toFixed(1);

    if (el.horizonDisc) {
      const ty = Math.max(-28, Math.min(28, state.calibPitch * 0.8));
      el.horizonDisc.style.transform = `rotate(${-state.calibRoll.toFixed(1)}deg) translateY(${ty.toFixed(1)}px)`;
    }
  }

  function updateImuMetrics(linAcc, accG, rotRate, intervalMs) {
    const lx = linAcc && typeof linAcc.x === 'number' ? linAcc.x : 0;
    const ly = linAcc && typeof linAcc.y === 'number' ? linAcc.y : 0;
    const lz = linAcc && typeof linAcc.z === 'number' ? linAcc.z : 0;

    const gx = accG && typeof accG.x === 'number' ? accG.x : 0;
    const gy = accG && typeof accG.y === 'number' ? accG.y : 0;
    const gz = accG && typeof accG.z === 'number' ? accG.z : 9.81;

    const rAlpha = rotRate && typeof rotRate.alpha === 'number' ? rotRate.alpha : 0;
    const rBeta = rotRate && typeof rotRate.beta === 'number' ? rotRate.beta : 0;
    const rGamma = rotRate && typeof rotRate.gamma === 'number' ? rotRate.gamma : 0;

    state.linAccX = lx;
    state.linAccY = ly;
    state.linAccZ = lz;
    state.accelX = gx;
    state.accelY = gy;
    state.accelZ = gz;
    state.gyroAlpha = rAlpha;
    state.gyroBeta = rBeta;
    state.gyroGamma = rGamma;
    if (typeof intervalMs === 'number' && intervalMs > 0) {
      state.imuIntervalMs = intervalMs;
    }

    const totalGravAccel = Math.sqrt(gx * gx + gy * gy + gz * gz);
    const gCur = totalGravAccel / 9.80665;
    const linMag =
      lx !== 0 || ly !== 0 || lz !== 0
        ? Math.sqrt(lx * lx + ly * ly + lz * lz)
        : Math.abs(totalGravAccel - 9.80665);

    state.gForceCur = gCur;
    state.linearAccelMs2 = linMag;
    if (gCur > state.gForceMax) {
      state.gForceMax = gCur;
    }

    if (el.gForceCur) el.gForceCur.textContent = state.gForceCur.toFixed(2);
    if (el.gForceMax) el.gForceMax.textContent = state.gForceMax.toFixed(2);
    if (el.accelMs2) el.accelMs2.textContent = state.linearAccelMs2.toFixed(2);

    if (el.linAccX) el.linAccX.textContent = lx.toFixed(2);
    if (el.linAccY) el.linAccY.textContent = ly.toFixed(2);
    if (el.linAccZ) el.linAccZ.textContent = lz.toFixed(2);

    if (el.accelX) el.accelX.textContent = gx.toFixed(2);
    if (el.accelY) el.accelY.textContent = gy.toFixed(2);
    if (el.accelZ) el.accelZ.textContent = gz.toFixed(2);

    if (el.gyroAlpha) el.gyroAlpha.textContent = rAlpha.toFixed(1);
    if (el.gyroBeta) el.gyroBeta.textContent = rBeta.toFixed(1);
    if (el.gyroGamma) el.gyroGamma.textContent = rGamma.toFixed(1);
    if (el.imuIntervalMs) {
      el.imuIntervalMs.textContent =
        state.imuIntervalMs !== null ? `${state.imuIntervalMs.toFixed(1)} ms` : '-- ms';
    }

    if (el.gForceDot) {
      const maxOffsetPx = 30;
      const dx = Math.max(-maxOffsetPx, Math.min(maxOffsetPx, (gx / 9.81) * 22));
      const dy = Math.max(-maxOffsetPx, Math.min(maxOffsetPx, (gy / 9.81) * 22));
      el.gForceDot.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
    }
  }

  function initDeviceSensors() {
    loadCalibration();

    const handleMotion = event => {
      if (state.isSimulating) return;
      updateImuMetrics(
        event.acceleration,
        event.accelerationIncludingGravity,
        event.rotationRate,
        event.interval
      );
    };

    const handleOrientation = event => {
      if (state.isSimulating) return;
      let yaw = null;
      if (typeof event.webkitCompassHeading === 'number') {
        yaw = event.webkitCompassHeading;
      } else if (typeof event.alpha === 'number') {
        yaw = (360 - event.alpha) % 360;
      }
      updateOrientationMetrics(yaw !== null ? yaw : state.rawYaw, event.beta, event.gamma);

      if (state.currentSpeedKmh < 2.0 && yaw !== null) {
        state.currentHeadingDeg = yaw;
        updateUI();
      }
    };

    if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
      el.btnEnableMotion.textContent = 'ĮJUNGTI JUTIKLIUS';
      el.btnEnableMotion.addEventListener('click', async () => {
        try {
          const resp = await DeviceMotionEvent.requestPermission();
          if (resp === 'granted') {
            window.addEventListener('devicemotion', handleMotion, { passive: true });
            window.addEventListener('deviceorientation', handleOrientation, { passive: true });
            el.btnEnableMotion.textContent = 'JUTIKLIAI: ĮJUNGTI';
          }
        } catch (_) {
          showAlert('Nepavyko aktyvuoti judesio jutiklių.');
        }
      });
    } else {
      window.addEventListener('devicemotion', handleMotion, { passive: true });
      if ('ondeviceorientationabsolute' in window) {
        window.addEventListener('deviceorientationabsolute', handleOrientation, { passive: true });
      }
      window.addEventListener('deviceorientation', handleOrientation, { passive: true });
    }

    if (el.btnCalibratePitch) {
      el.btnCalibratePitch.addEventListener('click', calibratePitchZero);
    }
    if (el.btnResetCalib) {
      el.btnResetCalib.addEventListener('click', resetPitchCalibration);
    }
  }

  // =========================================================================
  // E4, E5, E6: BATTERY, NETWORK & HARDWARE DIAGNOSTICS + FPS
  // =========================================================================

  function initBatteryTelemetry() {
    if (!('getBattery' in navigator)) {
      if (el.batLevel) el.batLevel.textContent = 'N/A';
      if (el.batState) el.batState.textContent = 'Nepalaikoma';
      return;
    }

    navigator.getBattery().then(battery => {
      const renderBat = () => {
        const pct = Math.round(battery.level * 100);
        state.batteryCurPct = pct;
        if (state.batteryStartPct === null) {
          state.batteryStartPct = pct;
        }
        state.batteryCharging = battery.charging;

        if (el.batLevel) el.batLevel.textContent = `${pct}%`;
        if (el.batState) {
          el.batState.textContent = battery.charging ? '⚡ Kraunama' : '🔋 Iškraunama';
        }

        const deltaPct = pct - state.batteryStartPct;
        let remText = '--';
        if (battery.charging && Number.isFinite(battery.chargingTime) && battery.chargingTime > 0) {
          remText = `${Math.round(battery.chargingTime / 60)} min iki 100%`;
        } else if (!battery.charging && Number.isFinite(battery.dischargingTime) && battery.dischargingTime > 0) {
          remText = `~${Math.round(battery.dischargingTime / 60)} min`;
        }
        if (el.batDeltaNote) {
          el.batDeltaNote.textContent = `Δ kelionės: ${deltaPct >= 0 ? '+' : ''}${deltaPct}% · Likę: ${remText}`;
        }
      };

      renderBat();
      battery.addEventListener('levelchange', renderBat);
      battery.addEventListener('chargingchange', renderBat);
      battery.addEventListener('dischargingtimechange', renderBat);
    }).catch(() => {});
  }

  function updateSystemDiagnostics() {
    // E5: Network & Connection
    const conn = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
    const isOnline = navigator.onLine;
    if (el.netType) {
      const eff = conn && conn.effectiveType ? conn.effectiveType.toUpperCase() : '';
      el.netType.textContent = isOnline ? (eff ? `ONLINE (${eff})` : 'ONLINE') : 'OFFLINE';
    }
    if (el.netDownlink) {
      el.netDownlink.textContent =
        conn && typeof conn.downlink === 'number' ? `${conn.downlink} Mbps` : '-- Mbps';
    }
    if (el.netRttNote) {
      el.netRttNote.textContent =
        conn && typeof conn.rtt === 'number' ? `RTT vėlinimas: ${conn.rtt} ms` : `Būsena: ${isOnline ? 'Prisijungta' : 'Atsijungta'}`;
    }

    // E6: Hardware, Screen & Memory
    const cores = navigator.hardwareConcurrency || '--';
    const ram = navigator.deviceMemory ? `${navigator.deviceMemory} GB` : 'N/A';
    if (el.hwCoresRam) {
      el.hwCoresRam.textContent = `${cores} brand. / ${ram}`;
    }

    const dpr = (window.devicePixelRatio || 1).toFixed(1);
    if (el.hwScreenNote) {
      el.hwScreenNote.textContent = `Ekranas: ${window.screen.width}×${window.screen.height} (@${dpr}x)`;
    }

    if (el.diagWakeLock) {
      el.diagWakeLock.textContent = state.wakeLockSentinel ? 'AKTYVUS (LOCKED)' : 'NEAKTYVUS';
    }

    const angle =
      window.screen && window.screen.orientation && typeof window.screen.orientation.angle === 'number'
        ? window.screen.orientation.angle
        : window.orientation || 0;
    if (el.diagScreenAngle) {
      el.diagScreenAngle.textContent = `${angle}°`;
    }

    if (el.diagJsHeap) {
      if (performance && performance.memory && performance.memory.usedJSHeapSize) {
        const mb = (performance.memory.usedJSHeapSize / (1024 * 1024)).toFixed(1);
        el.diagJsHeap.textContent = `${mb} MB`;
      } else {
        el.diagJsHeap.textContent = 'N/A';
      }
    }
  }

  function initFpsCounter() {
    let frames = 0;
    let lastTime = performance.now();
    const loop = now => {
      frames += 1;
      if (now - lastTime >= 1000) {
        state.fps = Math.round((frames * 1000) / (now - lastTime));
        if (el.fpsBadge) el.fpsBadge.textContent = `${state.fps} FPS`;
        frames = 0;
        lastTime = now;
      }
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  // =========================================================================
  // SCREEN WAKE LOCK API
  // =========================================================================

  async function requestWakeLock() {
    if (!('wakeLock' in navigator)) {
      el.wakeLockText.textContent = 'WAKE: N/A';
      return;
    }
    try {
      state.wakeLockSentinel = await navigator.wakeLock.request('screen');
      state.wakeLockEnabled = true;
      el.wakeLockBtn.classList.add('active');
      el.wakeLockText.textContent = 'WAKE: ĮJ.';
      state.wakeLockSentinel.addEventListener('release', () => {
        if (!state.wakeLockEnabled) {
          el.wakeLockBtn.classList.remove('active');
          el.wakeLockText.textContent = 'WAKE: IŠJ.';
        }
      });
    } catch (_) {
      el.wakeLockText.textContent = 'WAKE: AUTO';
    }
  }

  async function releaseWakeLock() {
    state.wakeLockEnabled = false;
    if (state.wakeLockSentinel) {
      try {
        await state.wakeLockSentinel.release();
      } catch (_) {}
      state.wakeLockSentinel = null;
    }
    el.wakeLockBtn.classList.remove('active');
    el.wakeLockText.textContent = 'WAKE: IŠJ.';
  }

  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible' && state.wakeLockEnabled) {
      requestWakeLock();
    }
  });

  // =========================================================================
  // LOCALSTORAGE SESSION PERSISTENCE & EXPORT (.GPX / .JSON / .CSV)
  // =========================================================================

  function saveSessionToStorage() {
    try {
      const payload = {
        savedAt: Date.now(),
        totalElapsedSec: state.totalElapsedSec,
        movingElapsedSec: state.movingElapsedSec,
        maxSpeedKmh: state.maxSpeedKmh,
        totalDistanceM: state.totalDistanceM,
        elevationGainM: state.elevationGainM,
        elevationLossM: state.elevationLossM,
        points: state.points.slice(-3000)
      };
      localStorage.setItem(STORAGE_KEY, JSON.stringify(payload));
      if (el.storageStatusText) {
        el.storageStatusText.textContent = `Išsaugota: ${state.points.length} tšk. (${new Date().toLocaleTimeString('lt-LT')})`;
      }
    } catch (_) {}
  }

  function restoreSessionFromStorage() {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return;
      const data = JSON.parse(raw);
      if (!data || !Array.isArray(data.points) || data.points.length === 0) return;

      state.totalElapsedSec = data.totalElapsedSec || 0;
      state.movingElapsedSec = data.movingElapsedSec || 0;
      state.maxSpeedKmh = data.maxSpeedKmh || 0;
      state.totalDistanceM = data.totalDistanceM || 0;
      state.elevationGainM = data.elevationGainM || 0;
      state.elevationLossM = data.elevationLossM || 0;
      state.points = data.points;

      const lastPt = state.points[state.points.length - 1];
      state.currentLat = lastPt.lat;
      state.currentLon = lastPt.lon;
      state.currentAltitude = lastPt.alt;
      state.referenceAltitude = lastPt.alt;
      state.currentHeadingDeg = lastPt.heading;
      state.isRunning = true;
      state.isPaused = true;

      el.btnStartText.textContent = 'TĘSTI';
      el.sessionStatusText.textContent = 'SESIJA ATKURTA (PAUZĖ)';
      updateUI();
      updateMapWithPoint(lastPt.lat, lastPt.lon, lastPt.acc || 10);
      renderChart();
      showAlert(`Atkurta ankstesnė kelionės sesija (${state.points.length} GPS taškų, ${(state.totalDistanceM / 1000).toFixed(2)} km). Spauskite TĘSTI arba ATSTATYTI.`);
    } catch (_) {}
  }

  function downloadFile(content, filename, mimeType) {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  function exportGPX() {
    if (state.points.length === 0) {
      showAlert('Nėra užfiksuotų maršruto taškų eksportavimui. Pradėkite kelionę arba įjunkite DEMO.');
      return;
    }
    const isoNow = new Date().toISOString();
    const trkpts = state.points
      .map(p => {
        const timeIso = new Date(p.timestamp).toISOString();
        const eleTag = p.alt !== null && p.alt !== undefined ? `<ele>${p.alt}</ele>` : '';
        return `      <trkpt lat="${p.lat}" lon="${p.lon}">${eleTag}<time>${timeIso}</time><extensions><speed>${p.speedMs}</speed><course>${p.heading || 0}</course></extensions></trkpt>`;
      })
      .join('\n');

    const gpx = `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="BUBIAI.GPS Telemetrija - https://gps.bubiai.eu" xmlns="http://www.topografix.com/GPX/1/1">
  <metadata>
    <name>Bubiai GPS Maršrutas (${isoNow.slice(0, 10)})</name>
    <time>${isoNow}</time>
  </metadata>
  <trk>
    <name>Bubiai GPS Track</name>
    <trkseg>
${trkpts}
    </trkseg>
  </trk>
</gpx>`;

    const stamp = isoNow.replace(/[:.]/g, '-').slice(0, 19);
    downloadFile(gpx, `bubiai-gps-${stamp}.gpx`, 'application/gpx+xml');
  }

  function exportJSON() {
    if (state.points.length === 0) {
      showAlert('Nėra užfiksuotų maršruto taškų eksportavimui. Pradėkite kelionę arba įjunkite DEMO.');
      return;
    }
    const payload = {
      app: 'BUBIAI.GPS Telemetrija (https://gps.bubiai.eu)',
      exportedAt: new Date().toISOString(),
      summary: {
        distanceMeters: Number(state.totalDistanceM.toFixed(1)),
        distanceKm: Number((state.totalDistanceM / 1000).toFixed(3)),
        totalTimeSeconds: Math.round(state.totalElapsedSec),
        movingTimeSeconds: Math.round(state.movingElapsedSec),
        maxSpeedKmh: Number(state.maxSpeedKmh.toFixed(2)),
        elevationGainM: Math.round(state.elevationGainM),
        elevationLossM: Math.round(state.elevationLossM),
        maxGForce: Number(state.gForceMax.toFixed(2)),
        totalPoints: state.points.length
      },
      points: state.points
    };
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    downloadFile(JSON.stringify(payload, null, 2), `bubiai-gps-${stamp}.json`, 'application/json');
  }

  function exportCSV() {
    if (state.points.length === 0) {
      showAlert('Nėra užfiksuotų maršruto taškų eksportavimui. Pradėkite kelionę arba įjunkite DEMO.');
      return;
    }
    const headers = [
      'timestamp',
      'iso_time',
      'lat',
      'lon',
      'alt_m',
      'speed_kmh',
      'speed_ms',
      'speed_knots',
      'doppler_ms',
      'haversine_kmh',
      'acc_h_m',
      'acc_v_m',
      'heading_deg',
      'distance_m',
      'gradient_pct',
      'vert_speed_m_min',
      'g_force',
      'pitch_deg',
      'roll_deg',
      'yaw_deg',
      'battery_pct',
      'dt_ms'
    ];
    const rows = state.points.map(p => [
      p.timestamp,
      new Date(p.timestamp).toISOString(),
      p.lat,
      p.lon,
      p.alt ?? '',
      p.speedKmh ?? 0,
      p.speedMs ?? 0,
      p.speedKnots ?? 0,
      p.dopplerMs ?? '',
      p.haversineKmh ?? 0,
      p.acc ?? '',
      p.altAcc ?? '',
      p.heading ?? '',
      p.distM ?? 0,
      p.gradPct ?? 0,
      p.vertMmin ?? 0,
      p.gForce ?? 1,
      p.pitch ?? 0,
      p.roll ?? 0,
      p.yaw ?? 0,
      p.batPct ?? '',
      p.dtMs ?? 0
    ].join(','));

    const csv = [headers.join(','), ...rows].join('\n');
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    downloadFile(csv, `bubiai-gps-${stamp}.csv`, 'text/csv;charset=utf-8');
  }

  // =========================================================================
  // EVENT LISTENERS & INITIALIZATION
  // =========================================================================

  function bindEvents() {
    el.btnStart.addEventListener('click', startTracking);
    el.btnPause.addEventListener('click', pauseTracking);
    el.btnReset.addEventListener('click', resetTracking);
    el.btnSim.addEventListener('click', toggleSimulation);

    el.alertBannerClose.addEventListener('click', hideAlert);

    el.wakeLockBtn.addEventListener('click', () => {
      if (state.wakeLockEnabled) releaseWakeLock();
      else requestWakeLock();
    });

    el.contrastBtn.addEventListener('click', () => {
      document.body.classList.toggle('high-contrast');
      el.contrastBtn.classList.toggle('active');
    });

    if (el.btnCopyCoords) {
      el.btnCopyCoords.addEventListener('click', async () => {
        if (state.currentLat === null || state.currentLon === null) {
          showAlert('Koordinatės dar negautos iš GPS imtuvo.');
          return;
        }
        const text = `${state.currentLat.toFixed(6)}, ${state.currentLon.toFixed(6)} (${decimalToDMS(state.currentLat, true)} ${decimalToDMS(state.currentLon, false)})`;
        try {
          await navigator.clipboard.writeText(text);
          el.copyCoordsText.textContent = 'NUKOPIJUOTA!';
          setTimeout(() => {
            el.copyCoordsText.textContent = 'KOPIJUOTI';
          }, 1800);
        } catch (_) {
          showAlert(`Koordinatės: ${text}`);
        }
      });
    }

    el.btnCenterMap.addEventListener('click', () => {
      state.autoCenterMap = true;
      el.btnCenterMap.classList.add('active');
      if (map && state.currentLat !== null && state.currentLon !== null) {
        map.setView([state.currentLat, state.currentLon], Math.max(map.getZoom(), 15), { animate: true });
      }
    });

    el.btnToggleMapTheme.addEventListener('click', () => {
      if (!map) return;
      state.darkMapLayer = !state.darkMapLayer;
      if (state.darkMapLayer) {
        map.removeLayer(tileLayerLight);
        tileLayerDark.addTo(map);
        if (el.mapContainerWrap) el.mapContainerWrap.classList.add('map-dark-active');
        el.btnToggleMapTheme.textContent = 'SLUOKSNIS: TAMSUS';
      } else {
        map.removeLayer(tileLayerDark);
        tileLayerLight.addTo(map);
        if (el.mapContainerWrap) el.mapContainerWrap.classList.remove('map-dark-active');
        el.btnToggleMapTheme.textContent = 'SLUOKSNIS: OSM';
      }
    });

    if (el.btnExportGpx) el.btnExportGpx.addEventListener('click', exportGPX);
    if (el.btnExportJson) el.btnExportJson.addEventListener('click', exportJSON);
    if (el.btnExportCsv) el.btnExportCsv.addEventListener('click', exportCSV);

    window.addEventListener('resize', () => {
      renderChart();
      if (map) map.invalidateSize();
      updateSystemDiagnostics();
    });
  }

  function registerServiceWorker() {
    if ('serviceWorker' in navigator) {
      navigator.serviceWorker.register('/sw.js').then(reg => {
        reg.update();
      }).catch(() => {});
    }
  }

  function init() {
    initPwaInstall();
    initUiMode();
    initMap();
    bindEvents();
    initDeviceSensors();
    initBatteryTelemetry();
    initFpsCounter();
    updateSystemDiagnostics();
    restoreSessionFromStorage();
    startClockLoop();
    requestWakeLock();
    ensureLiveGeolocation();
    renderChart();
    registerServiceWorker();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
