<script setup lang="ts">
import { computed } from 'vue'
import type { ActivitySplit } from './detail-types'

const props = defineProps<{ activity: { distanceKm: number; splits: ActivitySplit[] }; expanded?: boolean }>()
const maxPace = computed(() => Math.max(1, ...props.activity.splits.map(split => Math.round(split.paceSeconds))))
const km = (value: number) => value.toLocaleString('zh-CN', { minimumFractionDigits: 2, maximumFractionDigits: 2 })
const note = computed(() => {
  if (!props.activity.splits.length) return ''
  const total = props.activity.splits.reduce((sum, split) => sum + split.distanceKm, 0)
  return Math.abs(total - props.activity.distanceKm) > Math.max(0.03, props.activity.distanceKm * 0.005)
    ? `COROS 总距离为 ${km(props.activity.distanceKm)} km，原始分段累计 ${km(total)} km。此处保留原始分段，不按总距离缩放。` : ''
})
function pace(value: number) {
  if (!value || !Number.isFinite(value)) return '—'
  const seconds = Math.round(value)
  return `${Math.floor(seconds / 60)}′${String(seconds % 60).padStart(2, '0')}″`
}
</script>

<template>
  <section class="run-km-section" :class="{ expanded }" aria-label="每公里分段">
    <div class="runner-splits-heading"><h3>每公里分段</h3><span>配速 / 心率</span></div>
    <p v-if="note" class="run-km-note">{{ note }}</p>
    <div v-if="activity.splits.length" class="runner-splits" :tabindex="expanded ? undefined : 0" :aria-label="expanded ? '每公里分段列表' : '每公里分段，内容可滚动'">
      <div v-for="split in activity.splits" :key="split.index" class="runner-split">
        <span class="runner-split-index">{{ split.index }}<small v-if="split.distanceKm < 0.95">{{ split.distanceKm.toFixed(2) }}km</small></span>
        <span class="runner-split-bar"><i :style="{ width: `${Math.min(100, Math.max(8, (2 - Math.round(split.paceSeconds) / maxPace) * 60))}%` }" /></span>
        <b>{{ pace(split.paceSeconds) }}</b><span class="runner-split-hr">{{ split.heartRate ?? '—' }}</span>
      </div>
    </div>
    <p v-else class="run-km-note">这次跑步暂未提供每公里分段。</p>
  </section>
</template>

<style scoped>
.run-km-section { margin-top: 25px; border-top: 1px solid var(--run-border); padding-top: 20px; }
.runner-splits-heading { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; gap: 12px; }
.runner-splits-heading h3 { font-size: 12px; font-weight: 500; margin: 0; }
.runner-splits-heading > span { font-size: 9px; color: var(--run-muted); }
.runner-splits { max-height: 290px; overflow: auto; padding-right: 5px; }
.runner-splits:focus-visible { outline: 2px solid var(--run-accent); outline-offset: 4px; }
.runner-split { display: grid; grid-template-columns: 56px minmax(0, 1fr) 52px 28px; align-items: center; gap: 9px; min-height: 29px; font-size: 10px; }
.runner-split-index { display: flex; align-items: center; gap: 4px; min-width: 0; color: var(--run-muted); }
.runner-split-index small { font-size: 8px; }
.runner-split-bar { min-width: 0; }
.runner-split-bar i { display: block; height: 10px; background: var(--run-accent); opacity: .7; border-radius: 2px; }
.runner-split b { text-align: right; font-weight: 500; font-variant-numeric: tabular-nums; }
.runner-split-hr { text-align: right; color: var(--run-muted); }
.run-km-note { margin: 14px 0 0; font-size: 10px; line-height: 1.7; color: var(--run-muted); }
.expanded { margin-top: 26px; padding-top: 22px; }
.expanded .runner-splits-heading h3 { font-size: 14px; font-weight: 600; }
.expanded .runner-splits-heading > span { font-size: 10px; }
.expanded .runner-splits { max-height: none; overflow: visible; padding-right: 0; }
.expanded .runner-split { grid-template-columns: 70px minmax(0, 1fr) 60px 34px; min-height: 34px; font-size: 12px; }
.expanded .runner-split-index small { font-size: 10px; }
@media (max-width: 760px) { .runner-splits { max-height: 310px; } }
@media (max-width: 430px) { .expanded .runner-split { grid-template-columns: 62px minmax(0, 1fr) 52px 28px; gap: 7px; font-size: 11px; } .expanded .runner-split-index small { font-size: 9px; } }
</style>
