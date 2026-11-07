<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from 'vue'
import * as L from 'leaflet'
import 'leaflet/dist/leaflet.css'

const props = defineProps<{ segments: [number, number][][] }>()
const element = ref<HTMLElement | null>(null)
const tilesFailed = ref(false)
let map: L.Map | undefined
let observer: ResizeObserver | undefined
let routeBounds: L.LatLngBounds | undefined
function resetRoute() {
  if (!map || !routeBounds || !element.value?.clientWidth || !element.value.clientHeight) return
  map.stop()
  map.invalidateSize({ pan: false })
  map.fitBounds(routeBounds, { padding: [24, 24], maxZoom: 16, animate: false })
}
onMounted(() => {
  if (!element.value) return
  // Avoid a pending zoom transition touching the map after the dialog closes.
  map = L.map(element.value, { scrollWheelZoom: false, zoomAnimation: false, fadeAnimation: false, markerZoomAnimation: false })
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19, attribution: '© <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap</a> contributors',
  }).on('tileerror', () => { tilesFailed.value = true }).addTo(map)
  const color = getComputedStyle(element.value).getPropertyValue('--run-accent').trim()
  const coordinates = props.segments.map(segment => segment.map(([lon, lat]) => L.latLng(lat, lon)))
  L.polyline(coordinates, { color: '#203d47', weight: 6, opacity: 0.85, interactive: false }).addTo(map)
  const lines = L.polyline(coordinates, { color, weight: 3, opacity: 1 }).addTo(map)
  routeBounds = lines.getBounds()
  let fitted = false
  const resize = () => {
    if (!map || !element.value?.clientWidth || !element.value.clientHeight) return
    map.invalidateSize({ pan: false })
    // Cached components can mount while their parent dialog is still hidden.
    // Fit the route only after showModal gives the map its actual dimensions.
    if (!fitted) { resetRoute(); fitted = true }
  }
  resize()
  observer = new ResizeObserver(resize)
  observer.observe(element.value)
})
onBeforeUnmount(() => { observer?.disconnect(); map?.remove(); map = undefined })
</script>

<template>
  <div class="run-map-wrap">
    <div class="run-map-frame">
      <div ref="element" class="run-map" role="region" aria-label="可缩放的跑步路线地图" />
      <button type="button" class="run-map-reset" aria-label="复位跑步路线地图，查看完整路线" @click="resetRoute">复位</button>
    </div>
    <p v-if="tilesFailed" class="run-map-warning" role="status">底图暂时无法加载，路线仍可查看。</p>
  </div>
</template>

<style scoped>
.run-map-wrap { position: relative; }
.run-map-frame { position: relative; isolation: isolate; }
.run-map { z-index: 0; height: 340px; width: 100%; border: 1px solid var(--run-border); border-radius: 7px; background: var(--run-fill); font-family: inherit; }
.run-map-reset { position: absolute; z-index: 1; top: 12px; right: 12px; min-height: 36px; padding: 6px 12px; border: 1px solid var(--run-border); border-radius: 5px; background: var(--run-surface); color: var(--run-text); font: inherit; font-size: 12px; cursor: pointer; }
.run-map-reset:hover { border-color: var(--run-accent); }
.run-map-reset:focus-visible { outline: 2px solid var(--run-accent); outline-offset: 3px; }
.run-map-warning { margin: 8px 0 0; color: var(--run-muted); font-size: 11px; }
.run-map :deep(.leaflet-control-attribution) { font-size: 10px; }
@media (max-width: 600px) { .run-map { height: 260px; } }
</style>
