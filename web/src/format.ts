export function formatNumber(value: number | undefined, digits = 3): string {
  return value === undefined || !Number.isFinite(value) ? '—' : value.toFixed(digits)
}

export function formatPercent(value: number | undefined): string {
  return value === undefined || !Number.isFinite(value) ? '—' : `${value.toFixed(1)}%`
}
