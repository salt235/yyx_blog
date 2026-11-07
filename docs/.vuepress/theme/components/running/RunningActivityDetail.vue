<script setup lang="ts">
import { computed, defineAsyncComponent, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { withBase } from 'vuepress/client'
import type { ActivityDetail, DetailActivity } from './detail-types'
import RunningSplits from './RunningSplits.vue'

const props = defineProps<{ activity: DetailActivity }>()
const RunningMap = defineAsyncComponent(() => import('./RunningMap.vue'))
const RunningCurves = defineAsyncComponent(() => import('./RunningCurves.vue'))
const detail = ref<ActivityDetail | null>(null)
const state = ref<'loading' | 'ready' | 'unavailable' | 'error'>('loading')
const dialog = ref<HTMLDialogElement | null>(null)
const isOpen = ref(false)
const mounted = ref(false)
let previousOverflow = ''
let opener: HTMLElement | null = null
let manifest: Promise<string[]> | undefined
const cache = new Map<string, ActivityDetail>()

function availableIds() {
  if (!manifest) manifest = fetch(withBase('/running-data/details/index.json')).then(async response => {
    if (response.status === 404) return []
    if (!response.ok) throw new Error('读取失败')
    const result = await response.json()
    if (result.version !== 1 || !Array.isArray(result.ids)) throw new Error('格式错误')
    return result.ids as string[]
  }).catch(error => { manifest = undefined; throw error })
  return manifest
}

const retry = ref(0)
onMounted(() => { mounted.value = true })
watch([() => props.activity.id, mounted, retry], async ([id, ready], _, onCleanup) => {
  if (!ready) return
  close()
  detail.value = null
  state.value = 'loading'
  let active = true
  const controller = new AbortController()
  onCleanup(() => { active = false; controller.abort() })
  try {
    const ids = await availableIds()
    if (!active) return
    if (!ids.includes(id as string)) { state.value = 'unavailable'; return }
    let result = cache.get(id as string)
    if (!result) {
      const response = await fetch(withBase(`/running-data/details/${id}.json`), { signal: controller.signal })
      if (!response.ok) throw new Error('读取失败')
      result = await response.json() as ActivityDetail
      if (result.version !== 1 || result.id !== id || !Array.isArray(result.series) || !Array.isArray(result.route?.segments)) throw new Error('格式错误')
      cache.set(id as string, result)
    }
    if (active) { detail.value = result; state.value = 'ready' }
  } catch { if (active) state.value = 'error' }
})

const routeMessage = computed(() => {
  if (props.activity.sportType === 101 || detail.value?.route.status === 'indoor') return '室内跑 · 无 GPS 轨迹'
  return '这次训练没有可展示的 GPS 轨迹'
})
const routePaths = computed(() => {
  const segments = detail.value?.route.segments ?? []
  const points = segments.flat()
  if (!points.length) return []
  const latitude = points.reduce((sum, point) => sum + point[1], 0) / points.length * Math.PI / 180
  const project = ([lon, lat]: number[]) => [lon * Math.cos(latitude), lat]
  const projected = segments.map(segment => segment.map(project))
  const all = projected.flat()
  const minX = Math.min(...all.map(p => p[0])), maxX = Math.max(...all.map(p => p[0]))
  const minY = Math.min(...all.map(p => p[1])), maxY = Math.max(...all.map(p => p[1]))
  const scale = Math.min(324 / Math.max(maxX - minX, 0.000001), 124 / Math.max(maxY - minY, 0.000001))
  return projected.map(segment => segment.map(([x, y], index) => `${index ? 'L' : 'M'}${(180 + (x - (minX + maxX) / 2) * scale).toFixed(2)},${(80 - (y - (minY + maxY) / 2) * scale).toFixed(2)}`).join(' '))
})
const extraMetrics = computed(() => {
  const metrics = detail.value?.metrics
  return [
    ['平均步幅', props.activity.strideMeters?.toFixed(2), 'm'],
    ['平均跑步功率', props.activity.powerWatts, 'W'],
    ['训练负荷', props.activity.trainingLoad, ''],
    ['最高心率', metrics?.maxHeartRate, 'bpm'],
    ['最高步频', metrics?.maxCadence, 'spm'],
    ['累计下降', metrics?.descentMeters, 'm'],
    ['平均触地时间', metrics?.groundContactMs, 'ms'],
    ['平均垂直振幅', metrics?.verticalOscillationCm, 'cm'],
    ['平均垂直步幅比', metrics?.verticalRatioPercent, '%'],
  ].filter(([, value]) => value !== null && value !== undefined)
})
function duration(seconds: number | null | undefined) {
  if (seconds === null || seconds === undefined) return '—'
  const value = Math.round(seconds)
  return `${Math.floor(value / 3600)}:${String(Math.floor(value % 3600 / 60)).padStart(2, '0')}:${String(value % 60).padStart(2, '0')}`
}
function pace(seconds: number) {
  const value = Math.round(seconds)
  return `${Math.floor(value / 60)}′${String(value % 60).padStart(2, '0')}″`
}
async function open() {
  opener = document.activeElement as HTMLElement
  previousOverflow = document.body.style.overflow
  isOpen.value = true
  await nextTick()
  dialog.value?.showModal()
  document.body.style.overflow = 'hidden'
}
function close() {
  if (!isOpen.value) return
  dialog.value?.close()
  isOpen.value = false
  document.body.style.overflow = previousOverflow
  opener?.focus({ preventScroll: true })
}
onBeforeUnmount(close)
</script>

<template>
  <div class="run-preview">
    <div class="run-preview-heading"><h3>路线预览</h3><span v-if="activity.sportType === 101">室内跑</span><span v-else-if="detail?.route.status === 'available'">{{ activity.distanceKm.toFixed(2) }} km</span></div>
    <p v-if="state === 'loading'" class="run-preview-state" role="status">正在读取路线…</p>
    <div v-else-if="state === 'error'" class="run-preview-state" role="alert">详情暂时无法加载。<button class="run-text-button" @click="retry++">重试</button></div>
    <template v-else-if="state === 'unavailable'">
      <p v-if="activity.sportType === 101" class="run-preview-state">{{ routeMessage }}</p>
      <p class="run-preview-note">这次训练的完整详情尚未补全，历史记录正在分批导入。</p>
    </template>
    <template v-else-if="detail">
      <svg v-if="routePaths.length" class="run-route-outline" viewBox="0 0 360 160" role="img" aria-label="本次跑步路线轮廓，北向上">
        <path v-for="(path, index) in routePaths" :key="index" :d="path" />
      </svg>
      <p v-else class="run-preview-state">{{ routeMessage }}</p>
      <div class="run-preview-footer"><span v-if="!routePaths.length">可查看训练曲线与指标</span><button class="run-open-detail" @click="open">查看完整详情 <span aria-hidden="true">↗</span></button></div>
    </template>

    <dialog v-if="isOpen && detail" ref="dialog" class="run-dialog" aria-labelledby="run-dialog-title" @cancel.prevent="close" @click="($event.target === dialog) && close()">
      <div class="run-dialog-content">
        <header class="run-dialog-heading">
          <div><p>{{ activity.date }} <span v-if="activity.location">· {{ activity.location }}</span></p><h2 id="run-dialog-title">{{ activity.name }} <span>{{ activity.distanceKm.toFixed(2) }}<small> km</small></span></h2></div>
          <button class="run-close" aria-label="关闭完整详情" autofocus @click="close">×</button>
        </header>
        <dl class="run-dialog-summary">
          <div><dt>用时</dt><dd>{{ duration(activity.durationSeconds) }}</dd></div>
          <div><dt>平均配速</dt><dd>{{ pace(activity.paceSeconds) }}<small>/km</small></dd></div>
          <div><dt>平均心率</dt><dd>{{ activity.heartRate ?? '—' }}<small>bpm</small></dd></div>
          <div><dt>平均步频</dt><dd>{{ activity.cadence ?? '—' }}<small>spm</small></dd></div>
        </dl>
        <section class="run-full-route" aria-label="完整路线地图">
          <div class="run-section-heading"><h3>跑步路线</h3><span v-if="detail.route.status === 'available'">北向上</span></div>
          <RunningMap v-if="detail.route.segments.length" :segments="detail.route.segments" />
          <p v-else class="run-no-route">{{ routeMessage }}</p>
        </section>
        <section class="run-full-curves" aria-label="训练曲线">
          <div class="run-section-heading"><h3>训练曲线</h3><span>按经过时间</span></div>
          <RunningCurves :series="detail.series" />
        </section>
        <RunningSplits :activity="activity" expanded />
        <section class="run-extra" aria-label="更多训练数据">
          <div class="run-section-heading"><h3>更多训练数据</h3></div>
          <dl class="run-extra-metrics"><div v-for="[label, value, unit] in extraMetrics" :key="String(label)"><dt>{{ label }}</dt><dd>{{ value }}<small>{{ unit }}</small></dd></div><div><dt>计时用时</dt><dd>{{ duration(detail.metrics.timerSeconds) }}</dd></div><div><dt>经过时间（含暂停）</dt><dd>{{ duration(detail.metrics.elapsedSeconds) }}</dd></div></dl>
        </section>
        <footer class="run-dialog-source"><span>数据来自 COROS · {{ activity.date }}</span><button class="run-open-detail" @click="close">关闭详情</button></footer>
      </div>
    </dialog>
  </div>
</template>

<style scoped>
.run-preview { margin-top: 24px; padding-top: 20px; border-top: 1px solid var(--run-border); }
.run-preview-heading, .run-preview-footer, .run-section-heading { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
.run-preview h3, .run-section-heading h3 { margin: 0; font-size: 12px; font-weight: 600; }
.run-preview-heading > span, .run-preview-footer > span, .run-section-heading > span { color: var(--run-muted); font-size: 10px; }
.run-preview-footer .run-open-detail { margin-left: auto; }
.run-route-outline { display: block; width: 100%; height: 160px; margin: 12px 0; border-radius: 6px; background: var(--run-fill); }
.run-route-outline path { fill: none; stroke: var(--run-accent); stroke-width: 2; stroke-linecap: round; stroke-linejoin: round; vector-effect: non-scaling-stroke; }
.run-preview-state, .run-no-route { margin: 14px 0; padding: 30px 12px; background: var(--run-fill); border-radius: 6px; text-align: center; color: var(--run-muted); font-size: 12px; }
.run-preview-note { margin: 12px 0 0; font-size: 11px; line-height: 1.8; color: var(--run-muted); }
.run-open-detail { display: flex; align-items: center; gap: 12px; padding: 8px 11px; border: 1px solid var(--run-border); border-radius: 5px; background: var(--run-soft); font-size: 11px; }
.run-open-detail:hover { border-color: var(--run-accent); }
.run-text-button { border: 0; background: transparent; text-decoration: underline; padding: 4px 10px; }
.run-dialog { width: min(960px, calc(100% - 48px)); max-width: none; max-height: calc(100dvh - 48px); padding: 0; border: 1px solid var(--run-border); border-radius: 12px; background: var(--run-surface); color: var(--run-text); overscroll-behavior: contain; }
.run-dialog::backdrop { background: rgb(0 0 0 / 55%); }
.run-dialog-content { padding: 28px 32px 22px; }
.run-dialog-heading { display: flex; justify-content: space-between; align-items: flex-start; gap: 20px; margin-bottom: 26px; }
.run-dialog-heading p { font-size: 12px; margin: 0 0 8px; color: var(--run-muted); }
.run-dialog .run-dialog-heading h2 { margin: 0; font-size: 23px; border: 0; padding: 0; font-weight: 600; }
.run-dialog-heading h2 > span { margin-left: 12px; font-variant-numeric: tabular-nums; }
.run-dialog-heading h2 small { font-size: 12px; color: var(--run-muted); font-weight: 400; }
.run-close { display: grid; place-items: center; flex-shrink: 0; width: 36px; height: 36px; border-radius: 50%; border: 1px solid var(--run-border); background: var(--run-fill); font-size: 24px; line-height: 1; }
.run-dialog-summary { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 18px; margin: 0 0 26px; padding-bottom: 22px; border-bottom: 1px solid var(--run-border); }
.run-dialog-summary dt { color: var(--run-muted); font-size: 10px; }
.run-dialog-summary dd { margin: 6px 0 0; font-size: 18px; font-variant-numeric: tabular-nums; }
.run-dialog-summary small { margin-left: 4px; color: var(--run-muted); font-size: 10px; }
.run-section-heading { margin-bottom: 14px; }
.run-dialog .run-section-heading h3 { font-size: 14px; }
.run-full-curves, .run-extra { margin-top: 26px; padding-top: 22px; border-top: 1px solid var(--run-border); }
.run-extra-metrics { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 22px 16px; margin: 22px 0; }
.run-extra-metrics dt { font-size: 11px; color: var(--run-muted); }
.run-extra-metrics dd { margin: 7px 0 0; font-size: 20px; font-variant-numeric: tabular-nums; }
.run-extra-metrics small { margin-left: 5px; font-size: 11px; color: var(--run-muted); }
.run-dialog-source { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-top: 24px; padding-top: 14px; border-top: 1px solid var(--run-border); color: var(--run-muted); font-size: 10px; }
@media (max-width: 600px) {
  .run-dialog { width: calc(100% - 20px); max-height: calc(100dvh - 20px); }
  .run-dialog-content { padding: 20px 16px; }
  .run-dialog .run-dialog-heading h2 { font-size: 20px; }
  .run-extra-metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .run-dialog-summary { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .run-preview-footer { flex-wrap: wrap; }
}
</style>
