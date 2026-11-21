const CACHE_NAME = 'bubiai-gps-v4.0';
const ASSETS = [
  '/',
  '/index.html',
  '/styles.css?v=4.0',
  '/app.js?v=4.0',
  '/manifest.json',
  '/icon-192.png',
  '/icon-512.png',
  '/icon.svg'
];

self.addEventListener('install', event => {
  self.skipWaiting();
  event.waitUntil(
    caches.open(CACHE_NAME).then(cache => cache.addAll(ASSETS))
  );
});

self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys().then(keys =>
      Promise.all(keys.filter(k => k !== CACHE_NAME).map(k => caches.delete(k)))
    ).then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', event => {
  if (event.request.method !== 'GET') return;

  // Network-First strategy for HTML, CSS, JS, and Manifest so updates appear immediately
  event.respondWith(
    fetch(event.request)
      .then(networkResp => {
        if (networkResp && networkResp.status === 200 && event.request.url.startsWith(self.location.origin)) {
          const copy = networkResp.clone();
          caches.open(CACHE_NAME).then(cache => cache.put(event.request, copy));
        }
        return networkResp;
      })
      .catch(() => caches.match(event.request))
  );
});
