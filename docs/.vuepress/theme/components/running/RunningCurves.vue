<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { init, use, type EChartsType } from 'echarts/core'
import { LineChart } from 'echarts/charts'
import { GridComponent, TooltipComponent } from 'echarts/components'
import { CanvasRenderer } from 'echarts/renderers'
import type { ActivityDetail } from './detail-types'

use([LineChart, GridComponent, TooltipComponent, CanvasRenderer])
const props = defineProps<{ series: ActivityDetail['series'] }>()
const metrics = [
  { key: 'heartRate', label: '心率', unit: 'bpm' },
  { key: 'elevation', label: '海拔', unit: 'm' },
  { key: 'cadence', label: '步频', unit: 'spm' },
  { key: 'power', label: '功率', unit: 'W' },
] as const
const selected = ref<(typeof metrics)[number]['key']>('heartRate')
const available = computed(() => metrics.filter(metric => props.series.some(sample => sample[metric.key] !== null)))
const metric = computed(() => metrics.find(m => m.key === selected.value)!)
const element = ref<HTMLElement | null>(null)
let chart: EChartsType | undefined
let observer: ResizeObserver | undefined
function pace(value: number) { const s = Math.round(value); return `${Math.floor(s / 60)}′${String(s % 60).padStart(2, '0')}″` }
function time(value: number) { const s = Math.round(value); return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}` }
function render() {
  if (!chart || !element.value) return
  const style = getComputedStyle(element.value)
  const accent = style.getPropertyValue('--run-accent').trim()
  const muted = style.getPropertyValue('--run-muted').trim()
  const border = style.getPropertyValue('--run-border').trim()
  const hasPace = props.series.some(s => s.pace !== null)
  chart.setOption({
    animation: false,
    textStyle: { color: muted, fontFamily: style.fontFamily },
    grid: [{ top: 28, left: 54, right: 15, height: '33%' }, { top: '61%', left: 54, right: 15, bottom: 30 }],
    tooltip: { trigger: 'axis', confine: true, renderMode: 'richText', textStyle: { fontFamily: style.fontFamily }, formatter: (items: any) => {
      const item = items[0]
      if (!item) return ''
      const value = item.value[1]
      return `${time(item.value[0])}\n${item.seriesName}  ${value === null ? '无采样' : item.seriesIndex === 0 ? `${pace(value)} /km` : `${value} ${metric.value.unit}`}`
    } },
    xAxis: [0, 1].map(index => ({ type: 'value', gridIndex: index, min: 0, max: props.series.at(-1)?.seconds ?? 1, axisLabel: { color: muted, fontSize: 10, formatter: time }, splitLine: { show: false }, axisTick: { show: false }, axisLine: { lineStyle: { color: border } } })),
    yAxis: [
      { type: 'value', gridIndex: 0, name: hasPace ? '配速 /km' : '配速（无采样）', inverse: true, scale: true, nameLocation: 'start', nameTextStyle: { color: muted, fontSize: 10 }, axisLabel: { color: muted, fontSize: 10, formatter: pace }, splitLine: { lineStyle: { color: border } } },
      { type: 'value', gridIndex: 1, name: `${metric.value.label} ${metric.value.unit}`, scale: true, nameTextStyle: { color: muted, fontSize: 10 }, axisLabel: { color: muted, fontSize: 10 }, splitLine: { lineStyle: { color: border } } },
    ],
    series: [
      { name: '配速', type: 'line', xAxisIndex: 0, yAxisIndex: 0, showSymbol: false, connectNulls: false, lineStyle: { color: accent, width: 1.5 }, itemStyle: { color: accent }, data: props.series.map(s => [s.seconds, s.pace]) },
      { name: metric.value.label, type: 'line', xAxisIndex: 1, yAxisIndex: 1, showSymbol: false, connectNulls: false, lineStyle: { color: accent, width: 1.5 }, itemStyle: { color: accent }, areaStyle: { color: accent, opacity: 0.08 }, data: props.series.map(s => [s.seconds, s[selected.value]]) },
    ],
  }, true)
}
onMounted(async () => {
  // Canvas text does not repaint automatically when a web font finishes loading.
  await document.fonts.ready
  if (!element.value) return
  if (available.value.length && !available.value.some(m => m.key === selected.value)) selected.value = available.value[0].key
  chart = init(element.value)
  render()
  observer = new ResizeObserver(() => chart?.resize())
  observer.observe(element.value)
})
watch(selected, render)
onBeforeUnmount(() => { observer?.disconnect(); chart?.dispose(); chart = undefined })
</script>

<template>
  <div class="run-curves">
    <div class="run-curve-tabs" role="group" aria-label="第二条曲线指标"><button v-for="item in available" :key="item.key" :aria-pressed="selected === item.key" @click="selected = item.key">{{ item.label }}</button></div>
    <div ref="element" class="run-curve-chart" role="img" :aria-label="`配速和${metric.label}随经过时间变化的曲线，可在曲线上移动查看读数`" />
    <p v-if="!available.length" class="run-curve-note">本次训练没有记录心率、海拔、步频或功率曲线。</p>
  </div>
</template>

<style scoped>
.run-curve-tabs { display: flex; gap: 7px; flex-wrap: wrap; }
.run-curve-tabs button { padding: 5px 12px; border: 1px solid var(--run-border); border-radius: 5px; background: transparent; color: var(--run-muted); font-size: 11px; }
.run-curve-tabs button[aria-pressed="true"] { background: var(--run-soft); color: var(--run-accent); border-color: var(--run-accent); }
.run-curve-chart { width: 100%; height: 340px; margin-top: 14px; }
.run-curve-note { font-size: 11px; color: var(--run-muted); }
</style>
