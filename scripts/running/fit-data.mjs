import { Decoder, Stream } from '@garmin/fitsdk'

const finite = value => typeof value === 'number' && Number.isFinite(value)
const positive = value => finite(value) && value > 0 ? value : null
const nonnegative = value => finite(value) && value >= 0 ? value : null
const round = (value, digits = 1) => finite(value) ? Number(value.toFixed(digits)) : null

export function thin(points, limit) {
  if (points.length <= limit) return points
  return Array.from({ length: limit }, (_, i) => points[Math.round(i * (points.length - 1) / (limit - 1))])
}

export function routeSegments(points, limit = 1200) {
  const groups = []
  let current = []
  const flush = () => { if (current.length > 1) groups.push(current); current = [] }
  for (const point of points) {
    if (!point) { flush(); continue }
    current.push(point)
  }
  flush()
  const count = groups.reduce((sum, group) => sum + group.length, 0)
  // Preserve each segment's first and last point while keeping GPS gaps separate.
  return groups.map(group => thin(group, Math.max(2, Math.floor(limit * group.length / count))))
}

export function decodeFit(buffer) {
  const decoder = new Decoder(Stream.fromBuffer(buffer))
  if (!decoder.isFIT() || !decoder.checkIntegrity()) throw new Error('FIT 文件校验失败，未生成公开详情。')
  const { messages, errors } = decoder.read()
  if (errors.length || !messages.recordMesgs?.length || messages.sessionMesgs?.length !== 1) throw new Error('FIT 解码失败或包含多次运动，未生成公开详情。')
  return messages
}

export function normalizeFit(messages, activity) {
  const session = messages.sessionMesgs[0]
  if (session.sport !== 'running') throw new Error('FIT 不是跑步数据。')
  if (Math.abs(session.totalDistance / 1000 - activity.distanceKm) > Math.max(0.1, activity.distanceKm * 0.05)
    && activity.sportType !== 101) throw new Error('FIT 与活动总距离不符。')
  const records = messages.recordMesgs.filter(record => record.timestamp instanceof Date && Number.isFinite(+record.timestamp))
    .sort((a, b) => +a.timestamp - +b.timestamp)
  if (!records.length) throw new Error('FIT 没有有效采样时间。')
  const start = +(session.startTime ?? records[0].timestamp)
  if (Math.abs(+records[0].timestamp - start) > 86400000) throw new Error('FIT 采样时间异常。')
  const localDate = new Intl.DateTimeFormat('en-CA', { timeZone: 'Asia/Shanghai', year: 'numeric', month: '2-digit', day: '2-digit' }).format(new Date(start))
  if (localDate !== activity.date) throw new Error('FIT 与活动日期不符。')
  const positions = [], samples = []
  let previousTime = null
  for (const record of records) {
    const elapsed = (+record.timestamp - start) / 1000
    if (elapsed < 0 || (previousTime !== null && elapsed <= previousTime)) continue
    const gap = previousTime !== null && elapsed - previousTime > 15
    const lat = record.positionLat * 180 / 2 ** 31, lon = record.positionLong * 180 / 2 ** 31
    const gps = finite(lat) && finite(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180 && (lat !== 0 || lon !== 0)
    if (gap) positions.push(null)
    positions.push(gps ? [round(lon, 6), round(lat, 6)] : null)
    if (gap) samples.push({ seconds: round(previousTime + 0.1), pace: null, heartRate: null, elevation: null, cadence: null, power: null })
    const speed = record.enhancedSpeed ?? record.speed
    samples.push({
      seconds: round(elapsed),
      // Stationary samples have no finite pace; keep a visible gap rather than inventing a pace.
      pace: finite(speed) && speed > 0 ? round(1000 / speed) : null,
      heartRate: positive(record.heartRate),
      elevation: round(record.enhancedAltitude ?? record.altitude),
      // FIT running cadence counts cycles/min; COROS shows both feet as steps/min.
      cadence: round(nonnegative(record.cadence) === null ? null : (record.cadence + (record.fractionalCadence ?? 0)) * 2),
      power: nonnegative(record.power),
    })
    previousTime = elapsed
  }
  const route = activity.sportType === 101 ? [] : routeSegments(positions)
  const status = activity.sportType === 101 ? 'indoor' : route.length ? 'available' : 'missing'
  // Keep null boundaries when thinning so missing samples never turn into a continuous line.
  const step = Math.max(1, Math.ceil(samples.length / 1400))
  const series = samples.filter((sample, i) => i % step === 0 || i === samples.length - 1
    || ['pace', 'heartRate', 'elevation', 'cadence', 'power'].some(key => i > 0 && (sample[key] === null) !== (samples[i - 1][key] === null)))
  const max = key => { const values = samples.map(s => s[key]).filter(finite); return values.length ? Math.max(...values) : null }
  return {
    version: 1, id: activity.id, source: 'COROS FIT',
    route: { status, segments: route },
    series,
    metrics: {
      maxHeartRate: max('heartRate'), maxCadence: max('cadence'),
      elapsedSeconds: round(session.totalElapsedTime), timerSeconds: round(session.totalTimerTime),
      descentMeters: round(session.totalDescent),
      groundContactMs: round(positive(session.avgStanceTime)),
      verticalOscillationCm: round(positive(session.avgVerticalOscillation) === null ? null : session.avgVerticalOscillation / 10),
      verticalRatioPercent: round(positive(session.avgVerticalRatio)),
    },
  }
}
