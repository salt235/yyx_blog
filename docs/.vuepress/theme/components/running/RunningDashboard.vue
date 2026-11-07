<script setup lang="ts">
import { computed, nextTick, onMounted, ref, watch } from 'vue'
import { withBase } from 'vuepress/client'
import RunningActivityDetail from './RunningActivityDetail.vue'
import type { ActivitySplit as Split } from './detail-types'

interface Activity {
  id: string; date: string; name: string; sportType: number; location: string
  distanceKm: number; durationSeconds: number; paceSeconds: number
  heartRate: number | null; calories: number | null; cadence?: number | null
  strideMeters?: number | null; powerWatts?: number | null; elevationGainMeters?: number | null
  trainingLoad?: number | null; splits: Split[]
}
interface Dataset {
  version: number; source: string; updatedAt: string; timezone: string
  coverage: { start: string; end: string }; activities: Activity[]
}
const data = ref<Dataset | null>(null)
const loading = ref(true)
const error = ref('')
const period = ref('all')
const day = ref('')
const page = ref(1)
const selectedId = ref('')
const calendarMonth = ref('')
const monthListElement = ref<HTMLElement | null>(null)
const detailElement = ref<HTMLElement | null>(null)
const pageSize = 8

async function load() {
  loading.value = true
  error.value = ''
  try {
    const response = await fetch(withBase('/running-data/activities.json'), { cache: 'no-cache' })
    if (!response.ok) throw new Error('读取失败')
    const result = await response.json()
    if (result.version !== 1 || !Array.isArray(result.activities) || !result.coverage) throw new Error('格式错误')
    data.value = result
    calendarMonth.value = result.coverage.end.slice(0, 7)
    selectedId.value = result.activities[0]?.id ?? ''
  } catch {
    error.value = '暂时无法读取跑步记录，请稍后重试。'
  } finally { loading.value = false }
  await nextTick()
  if (monthListElement.value) monthListElement.value.scrollLeft = monthListElement.value.scrollWidth
}
onMounted(load)

const activities = computed(() => data.value?.activities ?? [])
const updatedDate = computed(() => data.value ? new Intl.DateTimeFormat('zh-CN', { timeZone: data.value.timezone, year: 'numeric', month: '2-digit', day: '2-digit' }).format(new Date(data.value.updatedAt)) : '')
const latest = computed(() => activities.value[0])
const months = computed(() => {
  if (!data.value) return []
  const first = new Date(`${data.value.coverage.start.slice(0, 7)}-01T00:00:00Z`)
  const end = data.value.coverage.end.slice(0, 7)
  const result: string[] = []
  for (let cursor = first; cursor.toISOString().slice(0, 7) <= end; cursor = new Date(Date.UTC(cursor.getUTCFullYear(), cursor.getUTCMonth() + 1, 1))) {
    result.push(cursor.toISOString().slice(0, 7))
  }
  return result.reverse()
})
const scope = computed(() => activities.value.filter(a => period.value === 'all' || a.date.startsWith(period.value)))
const distance = computed(() => scope.value.reduce((sum, a) => sum + a.distanceKm, 0))
const duration = computed(() => scope.value.reduce((sum, a) => sum + a.durationSeconds, 0))
const runningDays = computed(() => new Set(scope.value.map(a => a.date)).size)
const filtered = computed(() => scope.value.filter(a => !day.value || a.date === day.value))
const pages = computed(() => Math.max(1, Math.ceil(filtered.value.length / pageSize)))
const visible = computed(() => filtered.value.slice((page.value - 1) * pageSize, page.value * pageSize))
const selected = computed(() => filtered.value.find(a => a.id === selectedId.value) ?? null)
const dayTotals = computed(() => {
  const totals = new Map<string, number>()
  for (const a of activities.value) totals.set(a.date, (totals.get(a.date) ?? 0) + a.distanceKm)
  return totals
})
const calendar = computed(() => {
  if (!calendarMonth.value || !data.value) return []
  const [year, month] = calendarMonth.value.split('-').map(Number)
  const start = new Date(Date.UTC(year, month - 1, 1))
  const blanks = (start.getUTCDay() + 6) % 7
  const count = new Date(Date.UTC(year, month, 0)).getUTCDate()
  return Array.from({ length: blanks + count }, (_, index) => {
    const n = index - blanks + 1
    const key = `${calendarMonth.value}-${String(n).padStart(2, '0')}`
    return n < 1 ? null : { n, key, km: dayTotals.value.get(key) ?? 0,
      covered: key >= data.value!.coverage.start && key <= data.value!.coverage.end }
  })
})
const monthly = computed(() => [...months.value].reverse().map(month => ({
  month, km: activities.value.filter(a => a.date.startsWith(month)).reduce((sum, a) => sum + a.distanceKm, 0),
})))
const maxMonth = computed(() => Math.max(1, ...monthly.value.map(m => m.km)))
const monthlyAxis = computed(() => {
  const target = maxMonth.value / 5
  const magnitude = 10 ** Math.floor(Math.log10(target))
  const step = [1, 2, 5, 10].map(n => n * magnitude).find(n => n >= target)!
  const intervals = Math.ceil(maxMonth.value / step)
  return {
    max: intervals * step,
    ticks: Array.from({ length: intervals + 1 }, (_, i) => Number((i * step).toFixed(2))).reverse(),
  }
})
const calendarIndex = computed(() => months.value.indexOf(calendarMonth.value))
const selectedTitle = computed(() => selected.value ? `${selected.value.date.slice(5).replace('-', ' / ')} · ${selected.value.name}` : '选择一次跑步')

watch(period, () => { day.value = ''; page.value = 1 })
watch(filtered, records => {
  page.value = Math.min(page.value, Math.max(1, Math.ceil(records.length / pageSize)))
  if (!records.some(a => a.id === selectedId.value)) selectedId.value = records[0]?.id ?? ''
})
watch(period, async value => {
  if (value === 'all') return
  calendarMonth.value = value
  await nextTick()
  const list = monthListElement.value
  const row = list?.querySelector<HTMLElement>(`[data-month="${value}"]`)
  if (!list || !row) return
  const offset = row.getBoundingClientRect().left - list.getBoundingClientRect().left
  if (offset < 0 || offset + row.offsetWidth > list.clientWidth) {
    list.scrollLeft += offset - (list.clientWidth - row.offsetWidth) / 2
  }
})

function scrollMonths(event: WheelEvent) {
  const list = monthListElement.value
  if (!list || event.ctrlKey || event.shiftKey || Math.abs(event.deltaX) > Math.abs(event.deltaY)) return
  const unit = event.deltaMode === 1 ? 24 : event.deltaMode === 2 ? list.clientWidth : 1
  // The chart opens at the latest month on the right; wheel down reveals history.
  const next = Math.max(0, Math.min(list.scrollWidth - list.clientWidth, list.scrollLeft - event.deltaY * unit))
  if (next !== list.scrollLeft) {
    event.preventDefault()
    list.scrollLeft = next
  }
}

function pace(value: number | null | undefined) {
  if (!value || !Number.isFinite(value)) return '—'
  const seconds = Math.round(value)
  return `${Math.floor(seconds / 60)}′${String(seconds % 60).padStart(2, '0')}″`
}
function time(value: number) {
  const s = Math.round(value), h = Math.floor(s / 3600), m = Math.floor(s % 3600 / 60)
  return h ? `${h}:${String(m).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}` : `${m}:${String(s % 60).padStart(2, '0')}`
}
const km = (value: number) => value.toLocaleString('zh-CN', { minimumFractionDigits: 2, maximumFractionDigits: 2 })
const monthLabel = (value: string) => `${value.slice(0, 4)} 年 ${Number(value.slice(5))} 月`
const level = (value: number) => value >= 20 ? 4 : value >= 10 ? 3 : value > 5 ? 2 : value > 0 ? 1 : 0
function chooseDay(key: string) {
  if (day.value === key) { day.value = ''; return }
  period.value = 'all'
  nextTick(() => { day.value = key; page.value = 1 })
}
async function select(activity: Activity) {
  selectedId.value = activity.id
  await nextTick()
  if (window.matchMedia('(max-width: 760px)').matches) detailElement.value?.scrollIntoView({ behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'instant' : 'smooth', block: 'start' })
}
</script>

<template>
  <main class="runner" aria-label="yyx 的跑步记录">
    <header class="runner-heading">
      <div>
        <h1>我的跑步训练记录</h1>
        <p class="runner-intro">人生就像是一场马拉松，你可以不用跑得多快，但至少摇起来。</p>
      </div>
      <div v-if="latest" class="runner-latest">
        <span>最近一次 <b>{{ latest.date.slice(5).replace('-', ' / ') }}</b></span>
        <p>{{ km(latest.distanceKm) }}<small>km</small></p>
        <span>{{ latest.name }} <i>·</i> {{ time(latest.durationSeconds) }} <i>·</i> {{ pace(latest.paceSeconds) }}/km</span>
      </div>
    </header>

    <div v-if="loading" class="runner-state" role="status">正在读取跑步记录…</div>
    <div v-else-if="error" class="runner-state" role="alert"><p>{{ error }}</p><button class="runner-button" @click="load">重新加载</button></div>
    <template v-else-if="data">
      <section class="runner-stats" aria-label="所选范围统计">
        <div><span>累计距离</span><p>{{ km(distance) }} <small>km</small></p></div>
        <div><span>跑步次数</span><p>{{ scope.length }} <small>次</small></p></div>
        <div><span>跑步时间</span><p>{{ (duration / 3600).toFixed(1) }} <small>小时</small></p></div>
        <div><span>有跑步的日子</span><p>{{ runningDays }} <small>天</small></p></div>
      </section>

      <section class="runner-overview">
        <div class="runner-panel runner-volume">
          <div class="runner-panel-heading"><h2>月度跑量</h2><span>全部月份</span></div>
          <div class="runner-month-chart">
            <div class="runner-month-axis" aria-hidden="true">
              <span class="runner-month-unit">km</span>
              <div class="runner-month-scale"><span v-for="tick in monthlyAxis.ticks" :key="tick" :style="{ top: `${(1 - tick / monthlyAxis.max) * 100}%` }">{{ tick }}</span></div>
            </div>
            <div ref="monthListElement" class="runner-months" tabindex="0" role="region" aria-label="月度跑量，纵轴单位公里，可左右滚动浏览月份" @wheel="scrollMonths">
              <div class="runner-month-plot">
                <div class="runner-month-grid" aria-hidden="true"><i v-for="tick in monthlyAxis.ticks" :key="tick" :style="{ top: `${(1 - tick / monthlyAxis.max) * 100}%` }" /></div>
                <button v-for="month in monthly" :key="month.month" :data-month="month.month" class="runner-month" :class="{ active: period === month.month }" :aria-pressed="period === month.month" :aria-label="`${monthLabel(month.month)}，${km(month.km)} 公里，筛选该月`" @click="period = period === month.month ? 'all' : month.month">
                  <span class="runner-month-track"><span class="runner-month-column" :style="{ height: `${month.km / monthlyAxis.max * 100}%` }"><span class="runner-month-distance">{{ month.km.toFixed(1) }}</span></span></span>
                  <span class="runner-month-name">{{ month.month.slice(2).replace('-', ' / ') }}</span>
                </button>
              </div>
            </div>
          </div>
          <p class="runner-footnote">用滚轮或左右滑动查看月份，点击柱形查看记录。</p>
        </div>
        <div class="runner-panel runner-calendar">
          <div class="runner-panel-heading"><h2>训练日历</h2><div class="runner-calendar-nav"><button aria-label="上一个月" :disabled="calendarIndex >= months.length - 1" @click="calendarMonth = months[calendarIndex + 1]">‹</button><span>{{ monthLabel(calendarMonth) }}</span><button aria-label="下一个月" :disabled="calendarIndex <= 0" @click="calendarMonth = months[calendarIndex - 1]">›</button></div></div>
          <div class="runner-days"><span v-for="weekday in ['一', '二', '三', '四', '五', '六', '日']" :key="weekday" class="runner-weekday">{{ weekday }}</span>
            <template v-for="(cell, index) in calendar" :key="index"><span v-if="!cell" /><button v-else :disabled="!cell.covered || cell.km === 0" :class="[`runner-day-${level(cell.km)}`, { selected: day === cell.key, uncovered: !cell.covered }]" :title="`${cell.key} · ${!cell.covered ? '未同步' : cell.km ? `${km(cell.km)} km` : '无跑步记录'}`" :aria-label="`${cell.key}，${!cell.covered ? '未同步' : cell.km ? `${km(cell.km)} 公里` : '无跑步记录'}`" :aria-pressed="day === cell.key" @click="chooseDay(cell.key)">{{ cell.n }}</button></template>
          </div>
          <div class="runner-calendar-key"><span>少</span><i v-for="n in 5" :key="n" :class="`runner-day-${n - 1}`" /><span>多</span><span class="runner-key-tail">点击有跑步的日期</span></div>
        </div>
      </section>

      <section class="runner-log">
        <div class="runner-panel runner-records">
          <div class="runner-panel-heading runner-records-heading"><h2>训练详情</h2><span>{{ filtered.length }} 次跑步</span></div>
          <button v-if="day" class="runner-day-filter" @click="day = ''">{{ day }} <span>清除筛选 ×</span></button>
          <div v-if="!filtered.length" class="runner-empty">这个范围内还没有跑步记录。</div>
          <div v-else class="runner-record-list">
            <button v-for="activity in visible" :key="activity.id" class="runner-record" :class="{ selected: selectedId === activity.id }" :aria-pressed="selectedId === activity.id" :aria-label="`${activity.date} ${activity.name} ${km(activity.distanceKm)} 公里，查看详情`" @click="select(activity)">
              <span class="runner-record-date">{{ activity.date.slice(5).replace('-', '.') }}<small>{{ activity.date.slice(0, 4) }}</small></span>
              <span class="runner-record-description"><b>{{ activity.name }}</b><small>{{ activity.location || '跑步记录' }}</small></span>
              <span class="runner-record-distance">{{ km(activity.distanceKm) }}<small> km</small></span>
              <span class="runner-record-pace">{{ pace(activity.paceSeconds) }}<small>/km</small></span>
              <span class="runner-record-arrow" aria-hidden="true">↗</span>
            </button>
          </div>
          <nav v-if="filtered.length" class="runner-pagination" aria-label="跑步记录分页"><button :disabled="page <= 1" @click="page--">← 上一页</button><span>{{ page }} / {{ pages }}</span><button :disabled="page >= pages" @click="page++">下一页 →</button></nav>
          <footer class="runner-data-source">数据来自 COROS · 更新于 {{ updatedDate }}</footer>
        </div>

        <aside ref="detailElement" class="runner-panel runner-detail" aria-label="单次跑步详情" aria-live="polite">
          <div class="runner-panel-heading"><h2>{{ selectedTitle }}</h2><span v-if="selected">单次记录</span></div>
          <template v-if="selected">
            <p v-if="selected.location" class="runner-detail-location">{{ selected.location }}</p>
            <p class="runner-detail-distance">{{ km(selected.distanceKm) }}<small>km</small></p>
            <dl class="runner-detail-metrics">
              <div><dt>用时</dt><dd>{{ time(selected.durationSeconds) }}</dd></div>
              <div><dt>平均配速</dt><dd>{{ pace(selected.paceSeconds) }}<small>/km</small></dd></div>
              <div><dt>平均心率</dt><dd>{{ selected.heartRate ?? '—' }}<small>bpm</small></dd></div>
              <div><dt>平均步频</dt><dd>{{ selected.cadence ?? '—' }}<small>spm</small></dd></div>
              <div><dt>累计爬升</dt><dd>{{ selected.elevationGainMeters ?? '—' }}<small>m</small></dd></div>
              <div><dt>消耗</dt><dd>{{ selected.calories ?? '—' }}<small>kcal</small></dd></div>
            </dl>
            <RunningActivityDetail :activity="selected" />
          </template>
          <p v-else class="runner-empty">点击左侧记录，查看这次跑步。</p>
        </aside>
      </section>

    </template>

    <section class="runner-panel runner-stories">
      <div class="runner-panel-heading"><h2>跑步相关博客</h2></div>
      <div class="runner-story-links">
        <a :href="withBase('/blog/running-note-1.html')">
          <div class="runner-story-copy"><span>2026.03</span><b>跑步阶段小记：南京北岸半马</b></div><i aria-hidden="true">↗</i>
        </a>
        <a :href="withBase('/blog/run.html')">
          <div class="runner-story-copy"><span>2026.02</span><b>我的跑步历程</b></div><i aria-hidden="true">↗</i>
        </a>
      </div>
    </section>
  </main>
</template>

<style scoped>
@font-face { font-family: "Maple Mono CN"; src: url('/fonts/maple-mono/MapleMonoCN-Regular.woff2') format('woff2'); font-style: normal; font-weight: 400; font-display: swap; }
@font-face { font-family: "Maple Mono CN"; src: url('/fonts/maple-mono/MapleMonoCN-SemiBold.woff2') format('woff2'); font-style: normal; font-weight: 600; font-display: swap; }
.runner { --run-font-family: "Maple Mono", "Maple Mono CN", monospace; --run-accent: #337b95; --run-soft: #e9f2f5; --run-text: var(--vp-c-text-1); --run-muted: var(--vp-c-text-2); --run-border: var(--vp-c-divider); --run-surface: var(--vp-c-bg); --run-fill: var(--vp-c-bg-soft); max-width: 1160px; margin: 0 auto; padding: 60px 32px 72px; color: var(--run-text); font-family: var(--run-font-family); }
.runner :deep(*) { font-family: var(--run-font-family); }
[data-theme="dark"] .runner { --run-accent: #8cccd5; --run-soft: #253a42; }
.runner *, .runner *::before, .runner *::after { box-sizing: border-box; }
.runner button { font: inherit; color: inherit; }
.runner button { cursor: pointer; }
.runner button:disabled { cursor: default; }
.runner button:focus-visible, .runner a:focus-visible, .runner-months:focus-visible { outline: 2px solid var(--run-accent); outline-offset: 4px; }
.runner-heading { display: flex; align-items: center; justify-content: space-between; gap: 32px; padding-bottom: 38px; }
.runner h1 { margin: 0; font-size: clamp(32px, 4vw, 46px); font-weight: 650; letter-spacing: -.045em; line-height: 1.3; }
.runner-intro { margin: 17px 0 0; font-size: 13px; color: var(--run-muted); }
.runner-latest { min-width: 220px; border-left: 2px solid var(--run-accent); padding-left: 25px; }
.runner-latest > span { font-size: 11px; color: var(--run-muted); }
.runner-latest b { margin-left: 12px; color: var(--run-text); font-weight: 500; }
.runner-latest p { margin: 5px 0; font-size: 40px; line-height: 1.4; letter-spacing: -.06em; font-variant-numeric: tabular-nums; }
.runner-latest p small { margin-left: 9px; font-size: 15px; letter-spacing: 0; color: var(--run-muted); }
.runner-latest i { margin: 0 4px; font-style: normal; }
.runner-stats { display: grid; grid-template-columns: 1.35fr 1fr 1fr 1.1fr; padding: 26px 0 32px; border-top: 1px solid var(--run-border); border-bottom: 1px solid var(--run-border); }
.runner-stats > div { padding: 0 20px; border-left: 1px solid var(--run-border); }
.runner-stats > div:first-child { padding-left: 0; border: 0; }
.runner-stats > div > span { font-size: 11px; color: var(--run-muted); }
.runner-stats p { margin: 10px 0 0; font-size: clamp(24px, 3.1vw, 36px); font-weight: 500; letter-spacing: -.06em; line-height: 1.2; font-variant-numeric: tabular-nums; }
.runner-stats small { font-size: 11px; letter-spacing: 0; font-weight: 400; color: var(--run-muted); }
.runner-overview { display: grid; grid-template-columns: 1.2fr 1fr; gap: 24px; margin: 28px 0 40px; }
.runner-panel { border: 1px solid var(--run-border); border-radius: 10px; padding: 22px; background: var(--run-surface); min-width: 0; }
.runner-panel-heading { display: flex; justify-content: space-between; align-items: center; gap: 14px; }
.runner h2 { border: 0; padding: 0; margin: 0; font-size: 15px; font-weight: 600; line-height: 1.5; }
.runner-panel-heading > span { font-size: 10px; color: var(--run-muted); }
.runner-month-chart { --run-chart-height: 184px; display: grid; grid-template-columns: 34px minmax(0, 1fr); gap: 8px; margin-top: 16px; }
.runner-month-axis { position: relative; color: var(--run-muted); font-size: 10px; }
.runner-month-unit { position: absolute; top: 0; right: 6px; }
.runner-month-scale { position: relative; height: var(--run-chart-height); margin-top: 24px; }
.runner-month-scale span { position: absolute; right: 6px; transform: translateY(-50%); font-variant-numeric: tabular-nums; }
.runner-months { min-width: 0; overflow-x: auto; overflow-y: hidden; scrollbar-width: thin; scrollbar-color: var(--run-accent) var(--run-fill); }
.runner-month-plot { display: flex; position: relative; width: max-content; min-width: 100%; padding-top: 24px; }
.runner-month-grid { position: absolute; top: 24px; right: 0; left: 0; height: var(--run-chart-height); pointer-events: none; }
.runner-month-grid i { position: absolute; right: 0; left: 0; border-top: 1px solid var(--run-border); }
.runner-month { position: relative; display: flex; flex: 0 0 56px; flex-direction: column; align-items: center; width: 56px; padding: 0 6px; border: 0; border-radius: 4px; background: transparent; }
.runner-month:hover, .runner-month.active { background: var(--run-fill); }
.runner-month-name { height: 28px; line-height: 28px; font-size: 10px; white-space: nowrap; color: var(--run-muted); }
.runner-month-track { display: flex; align-items: flex-end; justify-content: center; width: 100%; height: var(--run-chart-height); }
.runner-month-column { position: relative; width: 24px; border-radius: 3px 3px 0 0; background: var(--run-accent); opacity: .75; }
.runner-month.active .runner-month-column { opacity: 1; }
.runner-month-distance { position: absolute; bottom: 100%; left: 50%; margin-bottom: 6px; transform: translateX(-50%); color: var(--run-text); font-size: 10px; white-space: nowrap; font-variant-numeric: tabular-nums; }
.runner-footnote { margin: 14px 0 0; font-size: 10px; line-height: 1.7; color: var(--run-muted); }
.runner-calendar-nav { display: flex; align-items: center; gap: 12px; font-size: 11px; }
.runner-calendar-nav button { width: 24px; height: 26px; border: 0; border-radius: 4px; background: var(--run-fill); font-size: 20px; line-height: 1; }
.runner-calendar-nav button:disabled { opacity: .3; }
.runner-days { display: grid; grid-template-columns: repeat(7, 1fr); gap: 5px; margin-top: 15px; }
.runner-weekday { padding: 0 0 4px; text-align: center; font-size: 10px; color: var(--run-muted); }
.runner-days button { height: 27px; border: 1px solid transparent; border-radius: 4px; font-size: 10px; }
.runner-days button.uncovered { background: transparent; opacity: .35; }
.runner-days button.selected { outline: 2px solid var(--run-text); outline-offset: 1px; }
.runner-day-0 { background: var(--run-fill); }
.runner-day-1 { background: color-mix(in srgb, var(--run-accent) 18%, var(--run-surface)); }
.runner-day-2 { background: color-mix(in srgb, var(--run-accent) 36%, var(--run-surface)); }
.runner-day-3 { background: color-mix(in srgb, var(--run-accent) 65%, var(--run-surface)); }
.runner-days .runner-day-4, .runner-day-4 { background: var(--run-accent); color: var(--run-surface); }
.runner-calendar-key { display: flex; align-items: center; gap: 4px; margin-top: 12px; font-size: 9px; color: var(--run-muted); }
.runner-calendar-key i { display: inline-block; height: 9px; width: 9px; border-radius: 2px; }
.runner-key-tail { margin-left: auto; }
.runner-log { display: grid; grid-template-columns: minmax(0, 1.35fr) minmax(0, 1fr); gap: 28px; align-items: start; }
.runner-records { min-width: 0; }
.runner-records-heading { padding: 0 0 18px; border-bottom: 1px solid var(--run-border); }
.runner-record { display: flex; align-items: center; gap: 13px; width: 100%; min-height: 82px; padding: 16px 10px; border: 0; border-bottom: 1px solid var(--run-border); background: transparent; text-align: left; transition: background 150ms; }
.runner-record:hover { background: var(--run-fill); }
.runner-record.selected { background: var(--run-soft); box-shadow: inset 3px 0 var(--run-accent); }
.runner-record-date { min-width: 44px; font-size: 14px; font-variant-numeric: tabular-nums; }
.runner-record small { display: block; margin-top: 5px; font-size: 10px; color: var(--run-muted); font-weight: 400; }
.runner-record-description { flex: 1; min-width: 0; }
.runner-record-description b { font-size: 12px; font-weight: 500; }
.runner-record-description small { white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.runner-record-distance { min-width: 66px; text-align: right; font-size: 17px; font-variant-numeric: tabular-nums; }
.runner-record-distance small { display: inline; font-size: 9px; }
.runner-record-pace { width: 54px; margin-left: 4px; text-align: right; font-size: 12px; }
.runner-record-arrow { margin-left: 2px; color: var(--run-accent); }
.runner-pagination { display: flex; justify-content: space-between; align-items: center; padding: 18px 0; font-size: 10px; color: var(--run-muted); }
.runner-pagination button { border: 0; padding: 5px 0; background: transparent; }
.runner-pagination button:disabled { opacity: .35; }
.runner-day-filter { display: flex; justify-content: space-between; width: 100%; margin-top: 10px; border: 0; border-radius: 5px; padding: 8px 10px; background: var(--run-soft); font-size: 11px !important; }
.runner-detail { position: sticky; top: calc(var(--vp-nav-height) + 20px); scroll-margin-top: calc(var(--vp-nav-height) + 20px); }
.runner-detail-location { margin: 10px 0 0; color: var(--run-muted); font-size: 11px; }
.runner-detail-distance { margin: 16px 0 24px; font-size: 48px; line-height: 1.2; letter-spacing: -.06em; font-variant-numeric: tabular-nums; }
.runner-detail-distance small { margin-left: 10px; color: var(--run-muted); font-size: 14px; letter-spacing: 0; }
.runner-detail-metrics { display: grid; grid-template-columns: repeat(2, 1fr); gap: 18px 12px; margin: 0; }
.runner-detail-metrics dt { color: var(--run-muted); font-size: 10px; }
.runner-detail-metrics dd { margin: 5px 0 0; font-size: 17px; font-variant-numeric: tabular-nums; }
.runner-detail-metrics small { margin-left: 4px; font-size: 9px; color: var(--run-muted); }
.runner-data-source { padding-top: 16px; font-size: 10px; line-height: 1.8; color: var(--run-muted); }
.runner-stories { margin-top: 32px; }
.runner-stories > .runner-panel-heading { flex-wrap: wrap; gap: 6px 14px; }
.runner-story-links { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; margin-top: 18px; }
.runner-story-links a { display: flex; align-items: center; gap: 16px; min-height: 104px; padding: 18px; border: 1px solid var(--run-border); border-radius: 8px; background: var(--run-fill); color: var(--run-text); text-decoration: none; }
.runner-story-links a:hover { border-color: var(--run-accent); background: var(--run-soft); }
.runner-story-copy { flex: 1; min-width: 0; }
.runner-story-links span { display: block; font-size: 11px; color: var(--run-muted); }
.runner-story-links b { display: block; margin-top: 8px; font-size: 16px; font-weight: 600; line-height: 1.6; }
.runner-story-links i { flex-shrink: 0; font-size: 22px; font-style: normal; color: var(--run-accent); }
.runner-state, .runner-empty { padding: 48px 16px; font-size: 13px; line-height: 1.8; color: var(--run-muted); text-align: center; }
.runner-button { padding: 7px 15px; border: 1px solid var(--run-border); background: var(--run-surface); border-radius: 5px; }
@media (max-width: 960px) { .runner { padding: 40px 24px 50px; } .runner-panel { padding: 18px; } .runner-log { gap: 18px; } .runner-record { gap: 9px; } .runner-record-pace { display: none; } }
@media (max-width: 760px) { .runner { padding: 30px 20px 45px; } .runner-heading { align-items: start; padding-bottom: 27px; } .runner-latest { min-width: 145px; padding-left: 15px; } .runner-latest p { font-size: 32px; } .runner-latest > span:last-child { display: none; } .runner-intro { max-width: 240px; line-height: 1.8; } .runner-stats { grid-template-columns: repeat(2, 1fr); gap: 25px 0; } .runner-stats > div:nth-child(3) { padding-left: 0; border: 0; } .runner-stats p { font-size: 30px; } .runner-overview { grid-template-columns: 1fr; gap: 16px; margin-bottom: 30px; } .runner-days button { height: 32px; } .runner-log { grid-template-columns: 1fr; } .runner-record-pace { display: block; } .runner-detail { position: static; margin-top: 10px; } .runner-detail-metrics { grid-template-columns: repeat(3, 1fr); } .runner-detail-distance { font-size: 42px; } .runner-story-links { grid-template-columns: 1fr; gap: 12px; } }
@media (max-width: 640px) { .runner-heading { flex-direction: column; gap: 22px; } .runner-latest { display: flex; align-items: baseline; gap: 14px; border-left: 0; border-bottom: 1px solid var(--run-border); padding: 0 0 5px; width: 100%; } .runner-latest p { margin: 0 0 0 auto; font-size: 25px; } .runner-intro { max-width: none; } }
@media (max-width: 430px) { .runner h1 { font-size: 36px; } .runner-intro { font-size: 12px; } .runner-calendar-nav { gap: 8px; } .runner-record-distance { min-width: 59px; font-size: 16px; } .runner-record-pace { width: 48px; font-size: 11px; } .runner-record-arrow { display: none; } }
@media (prefers-reduced-motion: reduce) { .runner-record { transition: none; } }
</style>
