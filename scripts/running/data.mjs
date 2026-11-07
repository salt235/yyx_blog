// COROS MCP currently returns human-readable summaries and JSON lap groups.
// Keep this boundary strict: a changed upstream format must not publish bad data.
export function unwrap(result) {
  if (result?.isError) throw new Error('COROS MCP 返回了工具错误，原有数据保持不变。')
  let value = result?.result ?? result
  if (value?.isError) throw new Error('COROS MCP 返回了工具错误。')
  if (value?.content) value = value.content.filter(item => item.type === 'text').map(item => item.text).join('\n')
  for (let i = 0; i < 3 && typeof value === 'string'; i++) {
    try { value = JSON.parse(value) } catch { break }
  }
  return value
}

export function seconds(value) {
  const parts = value.trim().split(':').map(Number)
  if (![2, 3].includes(parts.length) || parts.some(n => !Number.isFinite(n) || n < 0)
    || parts.slice(1).some(n => n >= 60)) throw new Error(`无法解析时间：${value}`)
  return parts.reduce((total, part) => total * 60 + part, 0)
}

const number = value => Number.isFinite(Number(value)) && Number(value) > 0 && Number(value) !== 65535 ? Number(value) : null
const capture = (text, pattern) => text.match(pattern)?.[1]
const required = (value, field) => {
  if (value === undefined) throw new Error(`COROS 返回格式变化：缺少 ${field}`)
  return value
}

export function parseRecords(result) {
  const text = unwrap(result)
  if (typeof text !== 'string') throw new Error('无法识别 COROS 运动列表格式。')
  if (/^No sport records found from \d{4}-\d{2}-\d{2} to \d{4}-\d{2}-\d{2}\.$/.test(text.trim())) return []
  const count = Number(required(capture(text, /\((\d+) records?\)/), 'record count'))
  const blocks = [...text.matchAll(/^\d+\.\s+(.+?)\s+[—–]\s+(\d{4}-\d{2}-\d{2})\s*\n([\s\S]*?)(?=^\d+\.\s|$(?![\s\S]))/gm)]
  if (blocks.length !== count) throw new Error(`运动列表不完整：声明 ${count} 条，解析到 ${blocks.length} 条。`)
  return blocks.map(([, kind, date, body]) => {
    const identity = required(body.match(/LabelId:\s*(\d+)\s*\|\s*SportType:\s*(\d+)/), 'LabelId / SportType')
    const distance = required(body.match(/Distance:\s*([\d.]+)\s*(km|m)\b/), 'Distance')
    const durationSeconds = seconds(required(capture(body, /Duration:\s*([\d:]+)/), 'Duration'))
    const distanceKm = Number(distance[1]) / (distance[2] === 'm' ? 1000 : 1)
    if (!(distanceKm > 0 && durationSeconds > 0)) throw new Error('运动距离和时间必须为正数。')
    return {
      id: identity[1], // COROS IDs exceed Number.MAX_SAFE_INTEGER.
      date, sportType: Number(identity[2]),
      name: ({ 100: '户外跑', 101: '室内跑', 102: '越野跑', 103: '操场跑' })[identity[2]] ?? kind,
      location: capture(body, /^\s*Location:\s*(.+)$/m)?.trim() ?? '',
      distanceKm, durationSeconds,
      paceSeconds: capture(body, /Average Pace:\s*([\d:]+)\s*\/km/) ? seconds(capture(body, /Average Pace:\s*([\d:]+)\s*\/km/)) : durationSeconds / distanceKm,
      heartRate: number(capture(body, /Avg HR:\s*(\d+)/)),
      calories: number(capture(body, /Calories:\s*(\d+)/)),
    }
  })
}

export function parseDetail(result) {
  const text = unwrap(result)
  if (typeof text !== 'string' || !/Activity Details/.test(text)) throw new Error('无法识别 COROS 单次详情格式。')
  const gain = capture(text, /Elevation Gain \/ Loss:\s*([\d.]+)/) ?? capture(text, /Total Ascent:\s*([\d.]+)/)
  return {
    cadence: number(capture(text, /Average Cadence:\s*([\d.]+)/)),
    strideMeters: number(capture(text, /Average Stride Length:\s*([\d.]+)/)),
    powerWatts: number(capture(text, /Average Power:\s*([\d.]+)/)),
    elevationGainMeters: gain !== undefined && Number(gain) === 0 ? 0 : number(gain),
    trainingLoad: number(capture(text, /Training Load:\s*([\d.]+)/)),
  }
}

export function parseLaps(result, activity) {
  const data = unwrap(result)
  if (!Array.isArray(data?.lapGroups)) throw new Error('无法识别 COROS 分圈格式。')
  // Structured workouts may use type 2 for repetitions and zero-distance rests,
  // while their actual kilometre splits live in another group (e.g. type 10).
  const candidates = data.lapGroups.filter(item => item.lapDistance === 100000 && item.laps?.length)
  const kilometreGroup = item => item.laps.every(lap => lap.distance > 0 && lap.distance <= 100001 && lap.time > 0)
  const group = candidates.find(item => item.type === 2 && kilometreGroup(item))
    ?? candidates.find(kilometreGroup) ?? candidates.find(item => item.type === 2)
  if (!group) return []
  const laps = group.laps.map((lap, index) => {
    // Verified against recorded activity totals: distance is cm, time is seconds.
    const distanceKm = number(lap.distance) / 100000
    const durationSeconds = number(lap.time)
    if (!(distanceKm > 0 && durationSeconds > 0)) throw new Error('COROS 分圈距离或时间无效。')
    return {
      index: index + 1, distanceKm, durationSeconds,
      paceSeconds: durationSeconds / distanceKm,
      heartRate: number(lap.avgHr), cadence: number(lap.avgCadence),
    }
  })
  const distance = laps.reduce((sum, lap) => sum + lap.distanceKm, 0)
  const duration = laps.reduce((sum, lap) => sum + lap.durationSeconds, 0)
  // COROS can expose an edited/calibrated summary alongside original lap data.
  // Validate laps against their own total group; never rescale their distances.
  const total = data.lapGroups.find(item => item.type === -1)?.laps?.[0]
  const referenceKm = total ? number(total.distance) / 100000 : activity.distanceKm
  const referenceSeconds = number(total?.time) ?? activity.durationSeconds
  const ratio = referenceKm / activity.distanceKm
  if (!(ratio > 0.5 && ratio < 2) || Math.abs(distance - referenceKm) > Math.max(0.03, referenceKm * 0.005)) {
    throw new Error('分圈与总距离不符，停止发布，请检查 COROS 的距离单位。')
  }
  // Some track runs omit the last few metres from the km group. Preserve the
  // recorded laps, allowing that short tail instead of inventing a split.
  const missingKm = Math.max(0, referenceKm - distance)
  const timeTolerance = 3 + missingKm * Math.max(0, ...laps.map(lap => lap.paceSeconds))
  if (Math.abs(duration - referenceSeconds) > timeTolerance || referenceSeconds / activity.durationSeconds > 2 || referenceSeconds / activity.durationSeconds < 0.5) throw new Error('分圈与总时间不符，停止发布，请检查 COROS 的时间单位。')
  return laps
}

export function makeDataset(activities, coverage) {
  if (!coverage || !/^\d{4}-\d{2}-\d{2}$/.test(coverage.start) || !/^\d{4}-\d{2}-\d{2}$/.test(coverage.end)) throw new Error('缺少同步范围。')
  return {
    version: 1, source: 'COROS MCP', timezone: 'Asia/Shanghai',
    updatedAt: new Date().toISOString(), coverage,
    activities: [...new Map(activities.map(activity => [activity.id, activity])).values()]
      .sort((a, b) => b.date.localeCompare(a.date) || b.id.localeCompare(a.id)),
  }
}
