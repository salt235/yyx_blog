export interface ActivitySplit {
  index: number; distanceKm: number; durationSeconds: number; paceSeconds: number
  heartRate: number | null; cadence: number | null
}
export interface DetailActivity {
  id: string; date: string; name: string; sportType: number; location: string
  distanceKm: number; durationSeconds: number; paceSeconds: number
  heartRate: number | null; cadence?: number | null; strideMeters?: number | null
  powerWatts?: number | null; trainingLoad?: number | null
  splits: ActivitySplit[]
}
export interface ActivityDetail {
  version: number; id: string; source: string
  route: { status: 'available' | 'indoor' | 'missing'; segments: [number, number][][] }
  series: { seconds: number; pace: number | null; heartRate: number | null; elevation: number | null; cadence: number | null; power: number | null }[]
  metrics: {
    maxHeartRate: number | null; maxCadence: number | null; elapsedSeconds: number | null
    timerSeconds: number | null; descentMeters: number | null; groundContactMs: number | null
    verticalOscillationCm: number | null; verticalRatioPercent: number | null
  }
}
