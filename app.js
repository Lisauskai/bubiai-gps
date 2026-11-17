/**
 * BUBIAI.GPS Telemetrija PWA v3.1
 * 2D Kalman Filter + Adaptive Accuracy (3 Tiers) + IMU Sensor Fusion + GNSS Wake-Up
 * https://gps.bubiai.eu
 */

(() => {
  'use strict';

  const STORAGE_KEY = 'bubiai_gps_session_v3';
  const MODE_STORAGE_KEY = 'bubiai_gps_ui_mode';
  const CALIB_STORAGE_KEY = 'bubiai_gps_calib_v1';

  const EARTH_RADIUS_M = 6371000;
  const MOVING_THRESHOLD_KMH = 1.5;

  // 3 Adaptive Accuracy Tiers
  const TIER_HIGH_ACC_M = 50;    // <= 50m: Aukštas tikslumas (Žalia)
  const TIER_MED_ACC_M = 500;    // 50m < acc <= 500m: Vidutinis tikslumas (Geltona)
                                 // > 500m: Žemas / Tinklo tikslumas (Oranžinė)

  const OSM_TILE_URL = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
  const CARTO_DARK_URL = 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png';
  const DEFAULT_CENTER = [55.8569, 23.1364];

  let deferredInstallPrompt = null;

  // =========================================================================
  // 2D KALMAN FILTER FOR GPS COORDINATES & VELOCITY
  // =========================================================================

  class GPSKalmanFilter {
    constructor() {
      this.reset();
    }

    reset() {
      this.lat = null;
      this.lon = null;
      this.variance = -1; // P (position variance in m^2)
      this.vLat = 0;      // deg/sec velocity
      this.vLon = 0;      // deg/sec velocity
      this.timestampMs = null;
      this.lastSpeedMs = 0;
    }

    /**
     * Process a new GPS measurement (lat, lon, accuracy, timestamp, speed)
     * Uses R = accuracy^2 as measurement noise covariance.
     */
    process(lat, lon, accuracy, timestampMs, dopplerSpeedMs = null, isStationaryImu = false) {
      const sigma = Math.max(accuracy || 10, 2.0);
      const R = sigma * sigma; // Measurement noise covariance R = accuracy^2

      if (this.variance < 0 || this.lat === null || this.lon === null) {
        this.lat = lat;
        this.lon = lon;
        this.variance = R;
        this.vLat = 0;
        this.vLon = 0;
        this.timestampMs = timestampMs;
        this.lastSpeedMs = dopplerSpeedMs !== null ? dopplerSpeedMs : 0;
        return {
          lat: this.lat,
          lon: this.lon,
          kalmanSpeedMs: this.lastSpeedMs,
          kalmanDeltaDistM: 0,
          dtSec: 0,
          gain: 1.0
        };
      }

      const prevLat = this.lat;
      const prevLon = this.lon;
      const dtSec = Math.max(0.1, (timestampMs - this.timestampMs) / 1000);
      this.timestampMs = timestampMs;

      // If IMU indicates device is completely stationary on a table, damp velocity vector
      if (isStationaryImu) {
        this.vLat *= 0.15;
        this.vLon *= 0.15;
      }

      // 1. PREDICT STEP: project state ahead along velocity vector (up to 15s horizon)
      const predictDt = Math.min(dtSec, 15.0);
      const predLat = this.lat + this.vLat * predictDt * 0.65;
      const predLon = this.lon + this.vLon * predictDt * 0.65;

      // Process noise Q (m/s): higher when moving fast, lower when stationary
      const qMetersPerSec = isStationaryImu
        ? 0.5
        : Math.max(3.0, dopplerSpeedMs !== null ? dopplerSpeedMs * 0.6 : this.lastSpeedMs * 0.5 + 3.5);
      this.variance += dtSec * qMetersPerSec * qMetersPerSec;

      // 2. UPDATE STEP: Kalman Gain K = P / (P + R)
      const K = this.variance / (this.variance + R);

      // Blend predicted position with measurement using Kalman gain K
      // When accuracy is small (5m -> R=25), K ~ 1 (trusts new point).
      // When accuracy is large (300m -> R=90000), K is smaller (smooths jumps).
      const newLat = predLat + K * (lat - predLat);
      const newLon = predLon + K * (lon - predLon);

      // Update velocity vector (deg/sec) with exponential smoothing
      const instVLat = (newLat - prevLat) / dtSec;
      const instVLon = (newLon - prevLon) / dtSec;
      const vAlpha = Math.max(0.25, Math.min(0.85, K + 0.15));
      this.vLat = this.vLat * (1 - vAlpha) + instVLat * vAlpha;
      this.vLon = this.vLon * (1 - vAlpha) + instVLon * vAlpha;

      this.lat = newLat;
      this.lon = newLon;
      this.variance = (1 - K) * this.variance;

      const kalmanDeltaDistM = haversineDistanceM(prevLat, prevLon, this.lat, this.lon);
      const kalmanSpeedMs = kalmanDeltaDistM / dtSec;
      this.lastSpeedMs = dopplerSpeedMs !== null && dopplerSpeedMs > 0 ? dopplerSpeedMs : kalmanSpeedMs;

      return {
        lat: this.lat,
        lon: this.lon,
        kalmanSpeedMs,
        kalmanDeltaDistM,
        dtSec,
        gain: K
      };
    }
  }

  const kalman = new GPSKalmanFilter();

  // Application State
  const state = {
    uiMode: 'basic',
    isRunning: false,
    isPaused: false,
    isSimulating: false,
    watchId: null,
    gnssPulseTimerId: null,
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
    rawUnfilteredDeltaM: 0,
    maxSpeedKmh: 0,
    speedWindowMs: [],

    // Distance
    totalDistanceM: 0,

    // Raw & Filtered Fix Tracking
    lastRawLogPoint: null,          // Always updated on every Raw Log entry (unfiltered)
    lastAcceptedFix: null,          // Updated on accepted fixes for adaptive distance/speed
    lastPacketWallMs: null,
    lastSub100mFixTimeMs: 0,        // For aggressive GNSS wake-up pulse (>100m for >10s)
    lastMedOrHighFixTimeMs: 0,      // Last fix with accuracy <= 500m (for 15s network fallback)
    lastDeltaMs: null,
    gpsHz: 0,
    gpsPacketsCount: 0,
    androidCoarseStreak: 0,         // Consecutive fixes with acc >= 200 && alt === null && speed === null

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
    kalmanLat: null,
    kalmanLon: null,
    horizAccuracyM: null,
    vertAccuracyM: null,
    accuracyTier: 'none', // 'high' | 'medium' | 'low'

    // 3D Orientation & Inclinometer
    rawPitch: 0,
    rawRoll: 0,
    rawYaw: 0,
    pitchOffset: 0,
    rollOffset: 0,
    calibPitch: 0,
    calibRoll: 0,

    // IMU & Sensor Fusion (Dead Reckoning / Stationary detection)
    imuListenersActive: false,
    gForceCur: 1.0,
    gForceMax: 1.0,
    linearAccelMs2: 0.0,
    imuEnergyWindow: [],
    imuVibrationRms: 0.0,
    isImuStationary: false,
    lastImuMotionTimeMs: Date.now(),
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
        } else if (el.installModal) {
          el.installModal.classList.remove('hidden');
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
  // MAP INITIALIZATION & ADAPTIVE ACCURACY STYLING (LEAFLET + OSM)
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
      opacity: 0.92,
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

  /**
   * Update map marker, accuracy circle, and polyline style according to accuracy tier:
   * - High (<=50m): solid bright green line (#00ffaa)
   * - Medium (50-500m): solid yellow/cyan line + lighter accuracy circle
   * - Low (>500m): dashed orange line
   */
  function updateMapWithPoint(lat, lon, accuracyM, tier = 'high') {
    if (!map) return;
    const latLng = [lat, lon];

    if (!map.hasLayer(currentMarker)) {
      currentMarker.addTo(map);
      accuracyCircle.addTo(map);
    }

    currentMarker.setLatLng(latLng);
    accuracyCircle.setLatLng(latLng);
    accuracyCircle.setRadius(Math.min(accuracyM || 10, 1200));

    if (tier === 'high') {
      routePolyline.setStyle({ color: '#00ffaa', dashArray: null, weight: 4 });
      accuracyCircle.setStyle({ color: '#00ffaa', fillColor: '#00ffaa', fillOpacity: 0.14 });
    } else if (tier === 'medium') {
      routePolyline.setStyle({ color: '#eab308', dashArray: null, weight: 3.5 });
      accuracyCircle.setStyle({ color: '#eab308', fillColor: '#eab308', fillOpacity: 0.08 });
    } else {
      routePolyline.setStyle({ color: '#f97316', dashArray: '8, 8', weight: 3 });
      accuracyCircle.setStyle({ color: '#f97316', fillColor: '#f97316', fillOpacity: 0.06 });
    }

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
  // GPS POSITION PROCESSING (KALMAN FILTER + 3 ADAPTIVE ACCURACY TIERS)
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

    const rawLat = coords.latitude;
    const rawLon = coords.longitude;
    const accuracy = typeof coords.accuracy === 'number' ? coords.accuracy : 9999;
    const alt = typeof coords.altitude === 'number' ? coords.altitude : null;
    const altAcc = typeof coords.altitudeAccuracy === 'number' ? coords.altitudeAccuracy : null;
    const nativeSpeedMs =
      typeof coords.speed === 'number' && !Number.isNaN(coords.speed) && coords.speed >= 0
        ? coords.speed
        : null;

    state.currentLat = rawLat;
    state.currentLon = rawLon;
    state.horizAccuracyM = accuracy;
    state.vertAccuracyM = altAcc;
    state.rawDopplerMs = nativeSpeedMs;

    if (accuracy <= 100) {
      state.lastSub100mFixTimeMs = nowMs;
    }

    // -----------------------------------------------------------------------
    // 1. UNFILTERED HAVERSINE Δd/Δt BETWEEN LAST TWO RAW LOG ENTRIES (ALWAYS!)
    // -----------------------------------------------------------------------
    let rawUnfilteredDistM = 0;
    let rawUnfilteredDtSec = 0;
    let rawUnfilteredSpeedMs = 0;

    if (state.lastRawLogPoint) {
      rawUnfilteredDistM = haversineDistanceM(
        state.lastRawLogPoint.lat,
        state.lastRawLogPoint.lon,
        rawLat,
        rawLon
      );
      rawUnfilteredDtSec = Math.max(0.2, (nowMs - state.lastRawLogPoint.timestamp) / 1000);
      rawUnfilteredSpeedMs = rawUnfilteredDistM / rawUnfilteredDtSec;
    }
    state.lastRawLogPoint = { lat: rawLat, lon: rawLon, timestamp: nowMs };

    const rawUnfilteredSpeedKmh = rawUnfilteredSpeedMs * 3.6;
    const nativeSpeedKmh = nativeSpeedMs !== null ? nativeSpeedMs * 3.6 : null;

    state.rawUnfilteredDeltaM = rawUnfilteredDistM;
    state.rawHaversineKmh = rawUnfilteredSpeedKmh;
    state.rawSpeedKmh = nativeSpeedKmh !== null ? nativeSpeedKmh : rawUnfilteredSpeedKmh;

    if (el.rawSpeedKmh) el.rawSpeedKmh.textContent = `${state.rawSpeedKmh.toFixed(1)} km/h`;
    if (el.rawGpsSpeedMs) {
      el.rawGpsSpeedMs.textContent =
        nativeSpeedMs !== null ? `${nativeSpeedMs.toFixed(2)} m/s` : 'null';
    }
    if (el.rawHaversineKmh) {
      el.rawHaversineKmh.textContent = `${rawUnfilteredSpeedKmh.toFixed(1)} km/h (Δd=${rawUnfilteredDistM.toFixed(1)}m)`;
    }

    // -----------------------------------------------------------------------
    // 2. ANDROID CHROME COARSE LOCATION DIAGNOSTIC CHECK
    // -----------------------------------------------------------------------
    if (accuracy >= 200 && alt === null && nativeSpeedMs === null) {
      state.androidCoarseStreak += 1;
      if (state.androidCoarseStreak >= 2) {
        showAlert(
          `⚠️ Android blokuoja tikslų GPS šiai naršyklei (gaunama tik ±${Math.round(accuracy)}m iš ryšio bokštų, be aukščio). Atidarykite telefono Nustatymai -> Programos -> Chrome -> Leidimai -> Vietovė ir įjunkite „Naudoti tikslią vietovę“ (Use precise location).`
        );
      }
    } else {
      state.androidCoarseStreak = 0;
      if (accuracy <= 100 && el.alertBannerText.textContent.includes('Android blokuoja tikslų GPS')) {
        hideAlert();
      }
    }

    // -----------------------------------------------------------------------
    // 3. 2D KALMAN FILTER PROCESSING
    // -----------------------------------------------------------------------
    const kf = kalman.process(rawLat, rawLon, accuracy, nowMs, nativeSpeedMs, state.isImuStationary);
    state.kalmanLat = kf.lat;
    state.kalmanLon = kf.lon;

    // -----------------------------------------------------------------------
    // 4. ADAPTIVE ACCURACY (3 TIERS: <=50m, 50-500m, >500m)
    // -----------------------------------------------------------------------
    let tier = 'high';
    if (accuracy <= TIER_HIGH_ACC_M) {
      tier = 'high';
      state.lastMedOrHighFixTimeMs = nowMs;
    } else if (accuracy <= TIER_MED_ACC_M) {
      tier = 'medium';
      state.lastMedOrHighFixTimeMs = nowMs;
    } else {
      tier = 'low';
    }
    state.accuracyTier = tier;

    // Displacement from last accepted session fix (both Kalman and Raw)
    let acceptedDeltaM = kf.kalmanDeltaDistM;
    let rawAcceptedDeltaM = rawUnfilteredDistM;
    let acceptedDtSec = kf.dtSec || rawUnfilteredDtSec || 1.0;

    if (state.lastAcceptedFix) {
      acceptedDeltaM = haversineDistanceM(
        state.lastAcceptedFix.kalmanLat,
        state.lastAcceptedFix.kalmanLon,
        kf.lat,
        kf.lon
      );
      rawAcceptedDeltaM = haversineDistanceM(
        state.lastAcceptedFix.rawLat,
        state.lastAcceptedFix.rawLon,
        rawLat,
        rawLon
      );
      acceptedDtSec = Math.max(0.25, (nowMs - state.lastAcceptedFix.timestamp) / 1000);
    }

    let shouldUpdateSpeedAndDist = false;
    let effectiveSpeedMs = 0;
    let stepDistM = 0;

    if (tier === 'high') {
      // TIER 1 (<= 50m): Full Doppler & Kalman/Haversine speed, every shift recorded
      shouldUpdateSpeedAndDist = true;
      if (nativeSpeedMs !== null && nativeSpeedMs > 0) {
        effectiveSpeedMs = nativeSpeedMs;
        el.speedSourceBadge.textContent = state.isSimulating ? 'SIMULIACIJA' : 'GPS DOPPLER (AUKŠTAS)';
      } else if (state.lastAcceptedFix && acceptedDeltaM >= 0.4) {
        const sp = acceptedDeltaM / acceptedDtSec;
        state.speedWindowMs.push(sp);
        if (state.speedWindowMs.length > 3) state.speedWindowMs.shift();
        effectiveSpeedMs = state.speedWindowMs.reduce((a, b) => a + b, 0) / state.speedWindowMs.length;
        el.speedSourceBadge.textContent = 'KALMAN + HAVERSINE';
      } else {
        effectiveSpeedMs = 0;
        el.speedSourceBadge.textContent = 'GPS (AUKŠTAS)';
      }
      stepDistM = acceptedDeltaM >= 0.4 ? acceptedDeltaM : 0;
    } else if (tier === 'medium') {
      // TIER 2 (50m < accuracy <= 500m): Points NOT rejected!
      // Speed and distance calculated via Kalman & Haversine if displacement > min(accuracy * 0.2, 30) m
      const minMoveGateM = Math.min(accuracy * 0.2, 30);
      const effectiveDisplacementM = Math.max(acceptedDeltaM, rawAcceptedDeltaM);

      if (!state.lastAcceptedFix) {
        shouldUpdateSpeedAndDist = true;
        effectiveSpeedMs = nativeSpeedMs !== null ? nativeSpeedMs : 0;
        stepDistM = 0;
      } else if (effectiveDisplacementM > minMoveGateM || (nativeSpeedMs !== null && nativeSpeedMs > 0.5)) {
        shouldUpdateSpeedAndDist = true;
        const calcSpeedMs =
          nativeSpeedMs !== null && nativeSpeedMs > 0
            ? nativeSpeedMs
            : acceptedDeltaM > 0.5
              ? acceptedDeltaM / acceptedDtSec
              : rawAcceptedDeltaM / acceptedDtSec;

        state.speedWindowMs.push(calcSpeedMs);
        if (state.speedWindowMs.length > 3) state.speedWindowMs.shift();
        effectiveSpeedMs = state.speedWindowMs.reduce((a, b) => a + b, 0) / state.speedWindowMs.length;
        stepDistM = acceptedDeltaM >= 1.0 ? acceptedDeltaM : rawAcceptedDeltaM;
      } else {
        // Point is still included in session, speed decays if no large move yet
        shouldUpdateSpeedAndDist = true;
        effectiveSpeedMs = state.isImuStationary ? 0 : state.currentSpeedMs * 0.7;
        stepDistM = 0;
      }
      el.speedSourceBadge.textContent = `KALMAN (±${Math.round(accuracy)}m)`;
    } else {
      // TIER 3 (accuracy > 500m): Low / Network accuracy
      // If no more accurate point in last 15 seconds, still calculate approximate Haversine speed
      const noBetterFixRecently = (nowMs - state.lastMedOrHighFixTimeMs) >= 15000;
      if (noBetterFixRecently) {
        shouldUpdateSpeedAndDist = true;
        if (state.lastAcceptedFix && (rawAcceptedDeltaM >= 2.0 || acceptedDeltaM >= 2.0)) {
          const netDistM = acceptedDeltaM >= 2.0 ? acceptedDeltaM : rawAcceptedDeltaM;
          effectiveSpeedMs = netDistM / acceptedDtSec;
          stepDistM = netDistM;
        } else {
          effectiveSpeedMs = state.isImuStationary ? 0 : state.currentSpeedMs * 0.5;
          stepDistM = 0;
        }
        el.speedSourceBadge.textContent = '(APYTIKSLIS / TINKLO)';
      }
    }

    if (shouldUpdateSpeedAndDist) {
      if (state.isImuStationary && nativeSpeedMs === null && stepDistM < 5.0) {
        effectiveSpeedMs = 0;
      }

      state.currentSpeedMs = effectiveSpeedMs;
      state.currentSpeedKmh = effectiveSpeedMs * 3.6;
      state.currentSpeedKnots = effectiveSpeedMs * 1.943844;

      // Heading / Compass
      if (typeof coords.heading === 'number' && !Number.isNaN(coords.heading) && coords.heading >= 0) {
        state.currentHeadingDeg = coords.heading;
      } else if (state.lastAcceptedFix && (acceptedDeltaM >= 1.5 || rawAcceptedDeltaM >= 2.0)) {
        state.currentHeadingDeg = computeBearingDeg(
          state.lastAcceptedFix.kalmanLat,
          state.lastAcceptedFix.kalmanLon,
          kf.lat,
          kf.lon
        );
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
          const dHoriz = haversineDistanceM(state.lastAltSample.lat, state.lastAltSample.lon, kf.lat, kf.lon);

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
          }
        }

        if (!state.lastAltSample || nowMs - state.lastAltSample.timeMs >= 2000) {
          state.lastAltSample = { alt, lat: kf.lat, lon: kf.lon, timeMs: nowMs };
        }
      }

      // Active Session Recording (Include High, Medium, and Active Low Fallback Points!)
      if (state.isRunning && !state.isPaused) {
        if (state.currentSpeedKmh > state.maxSpeedKmh) {
          state.maxSpeedKmh = state.currentSpeedKmh;
        }

        if (state.lastAcceptedFix && stepDistM > 0) {
          state.totalDistanceM += stepDistM;
        }

        state.points.push({
          lat: Number(kf.lat.toFixed(6)),
          lon: Number(kf.lon.toFixed(6)),
          rawLat: Number(rawLat.toFixed(6)),
          rawLon: Number(rawLon.toFixed(6)),
          alt: alt !== null ? Number(alt.toFixed(1)) : null,
          speedKmh: Number(state.currentSpeedKmh.toFixed(2)),
          speedMs: Number(state.currentSpeedMs.toFixed(2)),
          speedKnots: Number(state.currentSpeedKnots.toFixed(2)),
          dopplerMs: nativeSpeedMs !== null ? Number(nativeSpeedMs.toFixed(2)) : null,
          haversineKmh: Number(rawUnfilteredSpeedKmh.toFixed(2)),
          acc: Number(accuracy.toFixed(1)),
          tier,
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

      if (!state.lastAcceptedFix || stepDistM > 0 || tier === 'high') {
        state.lastAcceptedFix = {
          kalmanLat: kf.lat,
          kalmanLon: kf.lon,
          rawLat,
          rawLon,
          timestamp: nowMs,
          accuracy
        };
      }
    }

    const timeStr = new Date(gpsTimeMs).toLocaleTimeString('lt-LT');
    const dtLabel = state.lastDeltaMs !== null ? `${state.lastDeltaMs}ms` : '--ms';

    // Live Raw Log (Last 15 fixes)
    addDiagLog(
      `[${timeStr}] Lat:${rawLat.toFixed(5)} Lon:${rawLon.toFixed(5)} | Alt:${alt !== null ? alt.toFixed(0) + 'm' : '--'} | ${state.currentSpeedKmh.toFixed(1)} km/h (H:${rawUnfilteredSpeedKmh.toFixed(1)}) | ±${Math.round(accuracy)}m | Δt:${dtLabel}`,
      tier === 'high' ? 'normal' : 'coarse'
    );

    updateUI();
    updateMapWithPoint(kf.lat, kf.lon, accuracy, tier);
    renderChart();

    el.mapLastUpdate.textContent = `Atnaujinta: ${timeStr} (±${Math.round(accuracy)}m)`;
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

    // 3 Adaptive Accuracy Tiers Indicator (Green <=50m, Yellow 50-500m, Orange >500m)
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

      if (hAcc <= TIER_HIGH_ACC_M) {
        el.gpsQualityLabel.textContent = 'GPS: PUIKUS (≤50m)';
        el.gpsSignalDot.className = 'signal-dot good';
        el.gpsSignalBars.className = 'signal-bars bars-4';
      } else if (hAcc <= TIER_MED_ACC_M) {
        el.gpsQualityLabel.textContent = `GPS: VIDUTINIS (±${hAcc}m)`;
        el.gpsSignalDot.className = 'signal-dot medium';
        el.gpsSignalBars.className = 'signal-bars bars-3';
      } else {
        el.gpsQualityLabel.textContent = `TINKLO / ŽEMAS (±${hAcc}m)`;
        el.gpsSignalDot.className = 'signal-dot orange';
        el.gpsSignalBars.className = 'signal-bars bars-1';
      }
    }

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

    if (el.pointCount) el.pointCount.textContent = state.points.length.toString();
    el.mapPointsBadge.textContent = `${state.points.length} taškų`;
  }

  // =========================================================================
  // TIMERS, SENSOR FUSION DEAD RECKONING & AGGRESSIVE GNSS WAKE-UP PULSE
  // =========================================================================

  function startClockLoop() {
    if (state.clockTimerId) clearInterval(state.clockTimerId);
    state.lastTickTimestamp = Date.now();
    state.clockTimerId = setInterval(() => {
      updateSystemDiagnostics();
      const now = Date.now();

      // SENSOR FUSION: Speed handling between sparse GPS packets
      if (state.lastPacketWallMs !== null && now - state.lastPacketWallMs > 2500 && !state.isSimulating) {
        if (state.isImuStationary) {
          // Device is completely motionless on a table -> decay speed quickly to 0.0 km/h
          state.currentSpeedMs = state.currentSpeedMs < 0.15 ? 0 : state.currentSpeedMs * 0.45;
          state.currentSpeedKmh = state.currentSpeedMs * 3.6;
          state.currentSpeedKnots = state.currentSpeedMs * 1.943844;
        } else if (now - state.lastPacketWallMs <= 12000 && state.currentSpeedMs > 0) {
          // Device is vibrating/moving between delayed GPS updates -> gentle retention
          state.currentSpeedMs *= 0.96;
          state.currentSpeedKmh = state.currentSpeedMs * 3.6;
          state.currentSpeedKnots = state.currentSpeedMs * 1.943844;
        }
      }

      if (!state.isRunning || state.isPaused) {
        state.lastTickTimestamp = now;
        updateUI();
        return;
      }

      const dtSec = (now - state.lastTickTimestamp) / 1000;
      state.lastTickTimestamp = now;

      state.totalElapsedSec += dtSec;
      if (state.currentSpeedKmh >= MOVING_THRESHOLD_KMH) {
        state.movingElapsedSec += dtSec;
      }
      updateUI();
    }, 500);
  }

  /**
   * Aggressive GNSS Wake-Up Pulse:
   * If accuracy > 100m for more than 10 seconds, call one-shot getCurrentPosition
   * with enableHighAccuracy: true to force Android's hardware GNSS chip awake.
   */
  function startGnssWakeUpPulse() {
    if (state.gnssPulseTimerId) clearInterval(state.gnssPulseTimerId);
    state.lastSub100mFixTimeMs = Date.now();

    state.gnssPulseTimerId = setInterval(() => {
      if (state.isSimulating || !('geolocation' in navigator)) return;
      const now = Date.now();
      const isPoorAccuracy = state.horizAccuracyM === null || state.horizAccuracyM > 100;
      if (isPoorAccuracy && now - state.lastSub100mFixTimeMs >= 10000) {
        state.lastSub100mFixTimeMs = now;
        navigator.geolocation.getCurrentPosition(
          handlePosition,
          () => {},
          {
            enableHighAccuracy: true,
            maximumAge: 0,
            timeout: 10000
          }
        );
      }
    }, 5000);
  }

  function startTracking() {
    if (state.batteryCurPct !== null && state.batteryStartPct === null) {
      state.batteryStartPct = state.batteryCurPct;
    }

    // Automatically ensure IMU sensors are running on start
    ensureImuListenersActive();
    startGnssWakeUpPulse();

    if (state.isRunning && state.isPaused) {
      state.isPaused = false;
      state.lastTickTimestamp = Date.now();
      el.sessionStatusText.textContent = state.isSimulating ? 'DEMO VYKDOMA' : 'FIKSUOJAMA (GPS+IMU)';
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
    el.sessionStatusText.textContent = state.isSimulating ? 'DEMO VYKDOMA' : 'FIKSUOJAMA (GPS+IMU)';

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
    kalman.reset();

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
    state.lastAcceptedFix = null;
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
  // AUTOMATIC IMU STARTUP & SENSOR FUSION (ORIENTATION, ACCEL, GYRO)
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

    // Compute vibration energy RMS over rolling window for Stationary ZUPT detection
    state.imuEnergyWindow.push(linMag);
    if (state.imuEnergyWindow.length > 25) {
      state.imuEnergyWindow.shift();
    }
    const meanSq =
      state.imuEnergyWindow.reduce((sum, v) => sum + v * v, 0) / state.imuEnergyWindow.length;
    state.imuVibrationRms = Math.sqrt(meanSq);

    if (state.imuVibrationRms >= 0.14 || Math.abs(rAlpha) + Math.abs(rBeta) + Math.abs(rGamma) > 2.5) {
      state.lastImuMotionTimeMs = Date.now();
      state.isImuStationary = false;
    } else if (Date.now() - state.lastImuMotionTimeMs > 2000) {
      state.isImuStationary = true;
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

  function handleDeviceMotionEvent(event) {
    if (state.isSimulating) return;
    updateImuMetrics(
      event.acceleration,
      event.accelerationIncludingGravity,
      event.rotationRate,
      event.interval
    );
  }

  function handleDeviceOrientationEvent(event) {
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
  }

  function ensureImuListenersActive() {
    if (state.imuListenersActive) return;

    const needsIosPermission =
      typeof DeviceMotionEvent !== 'undefined' &&
      typeof DeviceMotionEvent.requestPermission === 'function';

    if (!needsIosPermission) {
      window.addEventListener('devicemotion', handleDeviceMotionEvent, { passive: true });
      if ('ondeviceorientationabsolute' in window) {
        window.addEventListener('deviceorientationabsolute', handleDeviceOrientationEvent, { passive: true });
      }
      window.addEventListener('deviceorientation', handleDeviceOrientationEvent, { passive: true });
      state.imuListenersActive = true;
      if (el.btnEnableMotion) {
        el.btnEnableMotion.textContent = 'IMU: AUTO AKTYVUS';
        el.btnEnableMotion.classList.add('active');
      }
    }
  }

  function initDeviceSensors() {
    loadCalibration();

    const needsIosPermission =
      typeof DeviceMotionEvent !== 'undefined' &&
      typeof DeviceMotionEvent.requestPermission === 'function';

    if (needsIosPermission) {
      if (el.btnEnableMotion) {
        el.btnEnableMotion.textContent = 'ĮJUNGTI JUTIKLIUS (iOS)';
        el.btnEnableMotion.addEventListener('click', async () => {
          try {
            const resp = await DeviceMotionEvent.requestPermission();
            if (resp === 'granted') {
              window.addEventListener('devicemotion', handleDeviceMotionEvent, { passive: true });
              window.addEventListener('deviceorientation', handleDeviceOrientationEvent, { passive: true });
              state.imuListenersActive = true;
              el.btnEnableMotion.textContent = 'JUTIKLIAI: ĮJUNGTI';
              el.btnEnableMotion.classList.add('active');
            }
          } catch (_) {
            showAlert('Nepavyko aktyvuoti judesio jutiklių.');
          }
        });
      }
    } else {
      // Android / Standard browsers: start IMU automatically on page load
      ensureImuListenersActive();
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
      updateMapWithPoint(lastPt.lat, lastPt.lon, lastPt.acc || 10, lastPt.tier || 'high');
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
      'tier',
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
      p.tier ?? '',
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
      navigator.serviceWorker.register('/sw.js?v=3.1').then(reg => {
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
    startGnssWakeUpPulse();
    renderChart();
    registerServiceWorker();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
