/**
 * BUBIAI GPS // Telemetrijos ir navigacijos PWA v2.0
 * https://gps.bubiai.eu
 */

(() => {
  'use strict';

  const STORAGE_KEY = 'bubiai_gps_session_v2';
  const EARTH_RADIUS_M = 6371000;
  const MOVING_THRESHOLD_KMH = 1.5; // Judėjimo laiko ir vid. judėjimo greičio slenkstis
  const MAX_USABLE_ACCURACY_M = 150; // Skaičiuoti greitį ir atstumą visiems taškams, kurių accuracy <= 150m
  const COARSE_LOCATION_THRESHOLD_M = 1000; // Apytikslės tinklo/IP vietos slenkstis (> 1000m)

  // Tile Layer URLs
  const OSM_TILE_URL = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
  const CARTO_DARK_URL = 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png';

  // Default coordinates: Bubiai, Šiaulių r.
  const DEFAULT_CENTER = [55.8569, 23.1364];

  // Application State
  const state = {
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
    rawSpeedKmh: 0,
    rawHaversineKmh: 0,
    maxSpeedKmh: 0,
    speedWindowMs: [],

    // Distance
    totalDistanceM: 0,

    // Last fix for Haversine delta calculation
    lastFix: null, // { lat, lon, timestamp, accuracy }

    // Altitude & Terrain
    currentAltitude: null,
    referenceAltitude: null,
    elevationGainM: 0,
    elevationLossM: 0,
    gradientPct: 0,
    verticalSpeedMmin: 0,
    lastAltSample: null,

    // Heading / Compass
    currentHeadingDeg: null,

    // Accuracy & Coordinates
    currentLat: null,
    currentLon: null,
    horizAccuracyM: null,
    vertAccuracyM: null,

    // G-Force & Accelerometer
    gForceCur: 1.0,
    gForceMax: 1.0,
    linearAccelMs2: 0.0,
    accelX: 0,
    accelY: 0,
    accelZ: 0,

    // Wake Lock
    wakeLockEnabled: true,
    wakeLockSentinel: null,

    // Map preferences
    autoCenterMap: true,
    darkMapLayer: true,

    // Diagnostic log counter & entries
    diagCount: 0,

    // Recorded track points
    points: []
  };

  // DOM Elements
  const el = {
    sessionStatusText: document.getElementById('sessionStatusText'),
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

    speedSourceBadge: document.getElementById('speedSourceBadge'),
    speedKmh: document.getElementById('speedKmh'),
    speedMs: document.getElementById('speedMs'),
    rawSpeedKmh: document.getElementById('rawSpeedKmh'),
    rawGpsSpeedMs: document.getElementById('rawGpsSpeedMs'),
    rawHaversineKmh: document.getElementById('rawHaversineKmh'),
    speedBarFill: document.getElementById('speedBarFill'),
    maxSpeedKmh: document.getElementById('maxSpeedKmh'),
    avgMovingSpeedKmh: document.getElementById('avgMovingSpeedKmh'),
    avgTotalSpeedKmh: document.getElementById('avgTotalSpeedKmh'),

    movingStateBadge: document.getElementById('movingStateBadge'),
    distancePrimary: document.getElementById('distancePrimary'),
    distanceUnit: document.getElementById('distanceUnit'),
    distanceMetersSub: document.getElementById('distanceMetersSub'),
    timerMoving: document.getElementById('timerMoving'),
    timerTotal: document.getElementById('timerTotal'),
    compassRing: document.getElementById('compassRing'),
    compassArrow: document.getElementById('compassArrow'),
    headingCardinal: document.getElementById('headingCardinal'),
    headingDeg: document.getElementById('headingDeg'),

    altFilterBadge: document.getElementById('altFilterBadge'),
    altitudeCur: document.getElementById('altitudeCur'),
    gradientPct: document.getElementById('gradientPct'),
    vertSpeedMmin: document.getElementById('vertSpeedMmin'),
    elevGain: document.getElementById('elevGain'),
    elevLoss: document.getElementById('elevLoss'),

    btnEnableMotion: document.getElementById('btnEnableMotion'),
    coordLat: document.getElementById('coordLat'),
    coordLon: document.getElementById('coordLon'),
    btnCopyCoords: document.getElementById('btnCopyCoords'),
    copyCoordsText: document.getElementById('copyCoordsText'),
    gForceDot: document.getElementById('gForceDot'),
    gForceCur: document.getElementById('gForceCur'),
    gForceMax: document.getElementById('gForceMax'),
    accelMs2: document.getElementById('accelMs2'),
    accelX: document.getElementById('accelX'),
    accelY: document.getElementById('accelY'),
    accelZ: document.getElementById('accelZ'),
    pointCount: document.getElementById('pointCount'),

    mapContainerWrap: document.getElementById('mapContainerWrap'),
    btnCenterMap: document.getElementById('btnCenterMap'),
    btnToggleMapTheme: document.getElementById('btnToggleMapTheme'),
    mapPointsBadge: document.getElementById('mapPointsBadge'),
    mapLastUpdate: document.getElementById('mapLastUpdate'),

    telemetryChart: document.getElementById('telemetryChart'),
    storageStatusText: document.getElementById('storageStatusText'),
    btnExportGpx: document.getElementById('btnExportGpx'),
    btnExportJson: document.getElementById('btnExportJson'),

    diagCountBadge: document.getElementById('diagCountBadge'),
    diagLogList: document.getElementById('diagLogList')
  };

  // Leaflet Map Instances
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

  /**
   * Haversine distance between two coordinates in meters
   */
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

  /**
   * Bearing (0-360°) between two coordinates
   */
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

  /**
   * Convert degrees (0-360) to Lithuanian cardinal direction
   */
  function degreesToCardinalLT(deg) {
    if (deg === null || deg === undefined || Number.isNaN(deg)) return '--';
    const dirs = ['Š', 'ŠR', 'R', 'PR', 'P', 'PV', 'V', 'ŠV'];
    const idx = Math.round(((deg % 360) / 45)) % 8;
    return dirs[idx];
  }

  /**
   * Format seconds into HH:MM:SS
   */
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
   * Append entry to the live Diagnostic Log (records all points including coarse >1000m)
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

    while (el.diagLogList.children.length > 40) {
      el.diagLogList.removeChild(el.diagLogList.lastChild);
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

    // Light / Standard Layer: Official OpenStreetMap URL
    tileLayerLight = L.tileLayer(OSM_TILE_URL, {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
      maxZoom: 19
    });

    // Dark Layer: Official OpenStreetMap tiles rendered with OLED Dark CSS filter
    // (Prevents CartoCDN "API KEY REQUIRED" watermark while keeping Carto config reference)
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
      color: '#00f5d4',
      weight: 4,
      opacity: 0.9,
      lineJoin: 'round'
    }).addTo(map);

    accuracyCircle = L.circle(DEFAULT_CENTER, {
      radius: 0,
      color: '#00f5d4',
      fillColor: '#00f5d4',
      fillOpacity: 0.12,
      weight: 1
    });

    const pulseIcon = L.divIcon({
      className: 'gps-map-marker',
      html: `<div style="width:14px;height:14px;border-radius:50%;background:#00f5d4;border:2px solid #000;box-shadow:0 0 12px #00f5d4;"></div>`,
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
    if (!canvas) return;
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

    // Subtle Grid
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

    // 1. Draw Altitude Area & Line (Green)
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

    // 2. Draw Speed Area & Line (Cyan)
    ctx.beginPath();
    recentPoints.forEach((p, i) => {
      const x = getX(i);
      const y = getYSpeed(p.speedKmh || 0);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = '#00f5d4';
    ctx.lineWidth = 2.2;
    ctx.stroke();

    ctx.lineTo(getX(recentPoints.length - 1), pad.top + plotH);
    ctx.lineTo(getX(0), pad.top + plotH);
    ctx.closePath();
    const gradSpd = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
    gradSpd.addColorStop(0, 'rgba(0, 245, 212, 0.28)');
    gradSpd.addColorStop(1, 'rgba(0, 245, 212, 0.0)');
    ctx.fillStyle = gradSpd;
    ctx.fill();

    // Axis Labels
    ctx.font = '600 9px "JetBrains Mono", monospace';
    ctx.fillStyle = '#00f5d4';
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
    const nowMs = Date.now(); // Use monotonic wall clock for reliable deltaTime
    const gpsTimeMs = pos.timestamp || nowMs;

    const lat = coords.latitude;
    const lon = coords.longitude;
    const accuracy = typeof coords.accuracy === 'number' ? coords.accuracy : 9999;
    const alt = typeof coords.altitude === 'number' ? coords.altitude : null;
    const altAcc = typeof coords.altitudeAccuracy === 'number' ? coords.altitudeAccuracy : null;
    const nativeSpeedMs = (typeof coords.speed === 'number' && !Number.isNaN(coords.speed) && coords.speed >= 0)
      ? coords.speed
      : null;

    state.currentLat = lat;
    state.currentLon = lon;
    state.horizAccuracyM = accuracy;
    state.vertAccuracyM = altAcc;

    // Calculate deltaDistance & deltaTime from previous fix
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

    // Update raw readout UI
    if (el.rawSpeedKmh) {
      el.rawSpeedKmh.textContent = `${state.rawSpeedKmh.toFixed(1)} km/h`;
    }
    if (el.rawGpsSpeedMs) {
      el.rawGpsSpeedMs.textContent = nativeSpeedMs !== null
        ? `coords.speed: ${nativeSpeedMs.toFixed(2)} m/s`
        : 'coords.speed: null';
    }
    if (el.rawHaversineKmh) {
      el.rawHaversineKmh.textContent = `${haversineSpeedKmh.toFixed(1)} km/h (Δd=${deltaDistM.toFixed(1)}m)`;
    }

    const timeStr = new Date(gpsTimeMs).toLocaleTimeString('lt-LT');

    // 1. COARSE LOCATION CHECK (coords.accuracy > 1000)
    if (accuracy > COARSE_LOCATION_THRESHOLD_M) {
      showAlert(
        `Gaunama tik apytikslė tinklo vieta (±${Math.round(accuracy)} m). Įjunkite „Tikslią vietovę“ (Precise Location) naršyklės leidimuose ir išeikite į atvirą vietą.`
      );
      addDiagLog(
        `[${timeStr}] APYTIKSLĖ VIETA: ${lat.toFixed(5)}, ${lon.toFixed(5)} | ±${Math.round(accuracy)}m | Raw: ${state.rawSpeedKmh.toFixed(1)} km/h`,
        'coarse'
      );
      updateUI();
      updateMapWithPoint(lat, lon, accuracy);
      el.mapLastUpdate.textContent = `Atnaujinta: ${timeStr} (±${Math.round(accuracy)}m)`;
      return;
    } else {
      // Clear coarse warning if accuracy improved <= 1000m
      if (el.alertBannerText.textContent.includes('apytikslė tinklo vieta')) {
        hideAlert();
      }
    }

    // 2. SPEED & DISTANCE CALCULATION FOR USABLE FIXES (coords.accuracy <= 150m)
    const isUsableAccuracy = accuracy <= MAX_USABLE_ACCURACY_M;
    let effectiveSpeedMs = 0;

    if (isUsableAccuracy) {
      // Use coords.speed if > 0; if null OR 0 while coordinates actually changed, use Haversine (deltaDistance / deltaTime)
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

      // Heading / Compass
      if (typeof coords.heading === 'number' && !Number.isNaN(coords.heading) && coords.heading >= 0) {
        state.currentHeadingDeg = coords.heading;
      } else if (state.lastFix && deltaDistM >= 1.5) {
        state.currentHeadingDeg = computeBearingDeg(state.lastFix.lat, state.lastFix.lon, lat, lon);
      }

      // Altimeter & Elevation Gain/Loss
      if (alt !== null) {
        state.currentAltitude = alt;
        const elevThreshold = Math.max(2.5, altAcc ? Math.min(8, altAcc * 0.45) : 2.8);
        el.altFilterBadge.textContent = `SLENKSTIS: ±${elevThreshold.toFixed(1)}m`;

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
          const dtMin = (nowMs - state.lastAltSample.timeMs) / 60000;
          const dAlt = alt - state.lastAltSample.alt;
          const dHoriz = haversineDistanceM(state.lastAltSample.lat, state.lastAltSample.lon, lat, lon);

          if (dtMin > 0.01) {
            state.verticalSpeedMmin = state.verticalSpeedMmin * 0.6 + (dAlt / dtMin) * 0.4;
          }
          if (dHoriz >= 3.0) {
            const rawGrad = (dAlt / dHoriz) * 100;
            const clampedGrad = Math.max(-45, Math.min(45, rawGrad));
            state.gradientPct = state.gradientPct * 0.65 + clampedGrad * 0.35;
          } else if (state.currentSpeedKmh < 1.0) {
            state.gradientPct = 0;
            state.verticalSpeedMmin = 0;
          }
        }

        if (!state.lastAltSample || nowMs - state.lastAltSample.timeMs >= 2000) {
          state.lastAltSample = { alt, lat, lon, timeMs: nowMs };
        }
      }

      // Active Trip Recording (Distance, Max Speed, Track Points)
      if (state.isRunning && !state.isPaused) {
        if (state.currentSpeedKmh > state.maxSpeedKmh) {
          state.maxSpeedKmh = state.currentSpeedKmh;
        }

        // Accumulate distance for all fixes with accuracy <= 150m when moving
        if (state.lastFix && deltaDistM >= 0.5 && (state.currentSpeedKmh >= 0.6 || deltaDistM >= 1.5)) {
          state.totalDistanceM += deltaDistM;
        }

        state.points.push({
          lat: Number(lat.toFixed(6)),
          lon: Number(lon.toFixed(6)),
          alt: alt !== null ? Number(alt.toFixed(1)) : null,
          speedKmh: Number(state.currentSpeedKmh.toFixed(2)),
          speedMs: Number(state.currentSpeedMs.toFixed(2)),
          rawSpeedKmh: Number(state.rawSpeedKmh.toFixed(2)),
          acc: Number(accuracy.toFixed(1)),
          altAcc: altAcc !== null ? Number(altAcc.toFixed(1)) : null,
          heading: state.currentHeadingDeg !== null ? Math.round(state.currentHeadingDeg) : null,
          distM: Number(state.totalDistanceM.toFixed(1)),
          timestamp: nowMs
        });
        saveSessionToStorage();
      }

      state.lastFix = { lat, lon, timestamp: nowMs, accuracy };
    }

    // Always log incoming point to Diagnostic List
    addDiagLog(
      `[${timeStr}] ${lat.toFixed(6)}, ${lon.toFixed(6)} | ±${Math.round(accuracy)}m | Greitis: ${state.currentSpeedKmh.toFixed(1)} km/h (Raw: ${state.rawSpeedKmh.toFixed(1)} km/h, Δd: ${deltaDistM.toFixed(1)}m)`,
      isUsableAccuracy ? 'normal' : 'coarse'
    );

    updateUI();
    updateMapWithPoint(lat, lon, accuracy);
    renderChart();

    el.mapLastUpdate.textContent = `Atnaujinta: ${timeStr}`;
  }

  /**
   * Handle watchPosition errors WITHOUT restarting watchPosition on TIMEOUT
   */
  function handlePositionError(err) {
    const timeStr = new Date().toLocaleTimeString('lt-LT');

    if (err.code === 3) {
      // TIMEOUT: DO NOT call clearWatch or restart watchPosition!
      // Let the active watchPosition continue acquiring satellites uninterrupted.
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
    el.maxSpeedKmh.textContent = state.maxSpeedKmh.toFixed(1);

    const barPct = Math.min(100, (state.currentSpeedKmh / 140) * 100);
    el.speedBarFill.style.width = `${barPct}%`;

    // Averages
    const avgTotal =
      state.totalElapsedSec > 2
        ? (state.totalDistanceM / 1000) / (state.totalElapsedSec / 3600)
        : 0;
    const avgMoving =
      state.movingElapsedSec > 2
        ? (state.totalDistanceM / 1000) / (state.movingElapsedSec / 3600)
        : 0;

    el.avgTotalSpeedKmh.textContent = avgTotal.toFixed(1);
    el.avgMovingSpeedKmh.textContent = avgMoving.toFixed(1);

    // 2. Distance & Timers
    if (state.totalDistanceM < 1000) {
      el.distancePrimary.textContent = Math.round(state.totalDistanceM).toString();
      el.distanceUnit.textContent = 'm';
      el.distanceMetersSub.textContent = `(${(state.totalDistanceM / 1000).toFixed(3)} km)`;
    } else {
      el.distancePrimary.textContent = (state.totalDistanceM / 1000).toFixed(2);
      el.distanceUnit.textContent = 'km';
      el.distanceMetersSub.textContent = `(${Math.round(state.totalDistanceM)} m)`;
    }

    el.timerTotal.textContent = formatDuration(state.totalElapsedSec);
    el.timerMoving.textContent = formatDuration(state.movingElapsedSec);

    const isMovingNow = state.currentSpeedKmh >= MOVING_THRESHOLD_KMH;
    el.movingStateBadge.textContent = isMovingNow ? 'JUDAMA' : 'STOVIMA';
    el.movingStateBadge.classList.toggle('moving', isMovingNow);

    // Compass
    if (state.currentHeadingDeg !== null) {
      const deg = Math.round(state.currentHeadingDeg) % 360;
      el.headingDeg.textContent = `${String(deg).padStart(3, '0')}°`;
      el.headingCardinal.textContent = degreesToCardinalLT(deg);
      el.compassArrow.style.transform = `rotate(${deg}deg)`;
    } else {
      el.headingDeg.textContent = '---°';
      el.headingCardinal.textContent = '--';
    }

    // 3. Altimeter
    el.altitudeCur.textContent =
      state.currentAltitude !== null ? state.currentAltitude.toFixed(1) : '--';
    el.elevGain.textContent = `+${Math.round(state.elevationGainM)}`;
    el.elevLoss.textContent = `-${Math.round(state.elevationLossM)}`;
    el.gradientPct.textContent = `${state.gradientPct >= 0 ? '+' : ''}${state.gradientPct.toFixed(1)}`;
    el.vertSpeedMmin.textContent = `${state.verticalSpeedMmin >= 0 ? '+' : ''}${state.verticalSpeedMmin.toFixed(1)}`;

    // 4. Signal Quality
    if (state.horizAccuracyM !== null) {
      const hAcc = Math.round(state.horizAccuracyM);
      const vAcc = state.vertAccuracyM !== null ? `±${Math.round(state.vertAccuracyM)}m` : '--m';
      el.accHoriz.textContent = `H: ±${hAcc}m`;
      el.accVert.textContent = `V: ${vAcc}`;

      if (hAcc <= 15) {
        el.gpsQualityLabel.textContent = 'GPS: PUIKUS';
        el.gpsSignalDot.className = 'signal-dot good';
        el.gpsSignalBars.className = 'signal-bars bars-4';
      } else if (hAcc <= MAX_USABLE_ACCURACY_M) {
        el.gpsQualityLabel.textContent = 'GPS: TINKAMAS (≤150m)';
        el.gpsSignalDot.className = 'signal-dot medium';
        el.gpsSignalBars.className = 'signal-bars bars-3';
      } else if (hAcc <= COARSE_LOCATION_THRESHOLD_M) {
        el.gpsQualityLabel.textContent = 'GPS: SILPNAS';
        el.gpsSignalDot.className = 'signal-dot';
        el.gpsSignalBars.className = 'signal-bars bars-2';
      } else {
        el.gpsQualityLabel.textContent = 'TINKLO / IP VIETA';
        el.gpsSignalDot.className = 'signal-dot';
        el.gpsSignalBars.className = 'signal-bars bars-1';
      }
    }

    // Coordinates
    if (state.currentLat !== null && state.currentLon !== null) {
      el.coordLat.textContent = `${state.currentLat.toFixed(6)}°`;
      el.coordLon.textContent = `${state.currentLon.toFixed(6)}°`;
    }

    // Points count
    el.pointCount.textContent = state.points.length.toString();
    el.mapPointsBadge.textContent = `${state.points.length} taškų`;
  }

  // =========================================================================
  // TIMERS & TRIP CONTROLS (START / PAUSE / RESET / DEMO)
  // =========================================================================

  function startClockLoop() {
    if (state.clockTimerId) clearInterval(state.clockTimerId);
    state.lastTickTimestamp = Date.now();
    state.clockTimerId = setInterval(() => {
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

  /**
   * Ensure single continuous watchPosition with enableHighAccuracy: true, timeout: 30000, maximumAge: 0.
   * Never recreates watchPosition if already active.
   */
  function ensureLiveGeolocation() {
    if (!('geolocation' in navigator)) {
      showAlert('Ši naršyklė nepalaiko Geolocation API.');
      return;
    }
    if (state.watchId !== null) {
      return; // Keep active watchPosition running without interruption
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
    state.verticalSpeedMmin = 0;
    state.lastAltSample = null;
    state.gForceMax = 1.0;
    state.points = [];

    localStorage.removeItem(STORAGE_KEY);
    if (routePolyline) routePolyline.setLatLngs([]);

    el.sessionStatusText.textContent = 'ATSTATYTA // PASIRUOŠĘS';
    el.btnStartText.textContent = 'PRADĖTI';
    el.btnStart.classList.remove('recording');
    el.btnPause.disabled = true;
    el.storageStatusText.textContent = 'Sesija išvalyta';

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
      updateGForceMetrics(gx, gy, 9.81 + Math.sin(simAngle) * 0.6);

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
  // DEVICE MOTION (ACCELEROMETER & G-FORCE)
  // =========================================================================

  function updateGForceMetrics(x, y, z) {
    state.accelX = x;
    state.accelY = y;
    state.accelZ = z;

    const totalAccel = Math.sqrt(x * x + y * y + z * z);
    const gCur = totalAccel / 9.80665;
    const dynAccel = Math.abs(totalAccel - 9.80665);

    state.gForceCur = gCur;
    state.linearAccelMs2 = dynAccel;
    if (gCur > state.gForceMax) {
      state.gForceMax = gCur;
    }

    el.gForceCur.textContent = state.gForceCur.toFixed(2);
    el.gForceMax.textContent = state.gForceMax.toFixed(2);
    el.accelMs2.textContent = state.linearAccelMs2.toFixed(1);
    el.accelX.textContent = x.toFixed(1);
    el.accelY.textContent = y.toFixed(1);
    el.accelZ.textContent = z.toFixed(1);

    const maxOffsetPx = 30;
    const dx = Math.max(-maxOffsetPx, Math.min(maxOffsetPx, (x / 9.81) * 22));
    const dy = Math.max(-maxOffsetPx, Math.min(maxOffsetPx, (y / 9.81) * 22));
    el.gForceDot.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
  }

  function initDeviceSensors() {
    const handleMotion = event => {
      if (state.isSimulating) return;
      const accG = event.accelerationIncludingGravity;
      if (!accG || (accG.x === null && accG.y === null)) return;
      updateGForceMetrics(accG.x || 0, accG.y || 0, accG.z || 9.81);
    };

    const handleOrientation = event => {
      if (state.currentSpeedKmh >= 2.0) return;
      let compassHeading = null;
      if (typeof event.webkitCompassHeading === 'number') {
        compassHeading = event.webkitCompassHeading;
      } else if (typeof event.alpha === 'number') {
        compassHeading = (360 - event.alpha) % 360;
      }
      if (compassHeading !== null) {
        state.currentHeadingDeg = compassHeading;
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
        } catch (e) {
          showAlert('Nepavyko aktyvuoti judesio jutiklių.');
        }
      });
    } else {
      window.addEventListener('devicemotion', handleMotion, { passive: true });
      window.addEventListener('deviceorientation', handleOrientation, { passive: true });
    }
  }

  // =========================================================================
  // SCREEN WAKE LOCK API
  // =========================================================================

  async function requestWakeLock() {
    if (!('wakeLock' in navigator)) {
      el.wakeLockText.textContent = 'EKRANAS: N/A';
      return;
    }
    try {
      state.wakeLockSentinel = await navigator.wakeLock.request('screen');
      state.wakeLockEnabled = true;
      el.wakeLockBtn.classList.add('active');
      el.wakeLockText.textContent = 'EKRANAS: ĮJ.';
      state.wakeLockSentinel.addEventListener('release', () => {
        if (!state.wakeLockEnabled) {
          el.wakeLockBtn.classList.remove('active');
          el.wakeLockText.textContent = 'EKRANAS: IŠJ.';
        }
      });
    } catch (err) {
      el.wakeLockText.textContent = 'EKRANAS: AUTO';
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
    el.wakeLockText.textContent = 'EKRANAS: IŠJ.';
  }

  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible' && state.wakeLockEnabled) {
      requestWakeLock();
    }
  });

  // =========================================================================
  // LOCALSTORAGE SESSION PERSISTENCE & EXPORT (.GPX / .JSON)
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
      el.storageStatusText.textContent = `Išsaugota: ${state.points.length} tšk. (${new Date().toLocaleTimeString('lt-LT')})`;
    } catch (e) {
      el.storageStatusText.textContent = 'Vietinės atminties limitas pasiektas';
    }
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
<gpx version="1.1" creator="Bubiai GPS Telemetry - https://gps.bubiai.eu" xmlns="http://www.topografix.com/GPX/1/1">
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
      app: 'Bubiai GPS Telemetry PWA (https://gps.bubiai.eu)',
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

    el.btnCopyCoords.addEventListener('click', async () => {
      if (state.currentLat === null || state.currentLon === null) {
        showAlert('Koordinatės dar negautos iš GPS imtuvo.');
        return;
      }
      const text = `${state.currentLat.toFixed(6)}, ${state.currentLon.toFixed(6)}`;
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

    el.btnExportGpx.addEventListener('click', exportGPX);
    el.btnExportJson.addEventListener('click', exportJSON);

    window.addEventListener('resize', () => {
      renderChart();
      if (map) map.invalidateSize();
    });
  }

  function registerServiceWorker() {
    if ('serviceWorker' in navigator) {
      navigator.serviceWorker.register('sw.js?v=2').then(reg => {
        reg.update();
      }).catch(() => {});
    }
  }

  function init() {
    initMap();
    bindEvents();
    initDeviceSensors();
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
