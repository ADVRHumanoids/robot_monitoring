<script setup lang="ts">
import { computed, ref } from 'vue'
import type { DiagnosticsSnapshot, DiagnosticTreeNode } from '../types'
import DiagnosticsTreeNode from './DiagnosticsTreeNode.vue'

type ExpansionMode = 'all' | 'warn-error' | 'error' | 'collapse'

const props = defineProps<{
  snapshot?: DiagnosticsSnapshot
  tree: DiagnosticTreeNode[]
}>()

const query = ref('')
const expansionMode = ref<ExpansionMode>('collapse')
const expansionRevision = ref(0)
const expandMenu = ref<HTMLDetailsElement>()
const filteredTree = computed(() => {
  const needle = query.value.trim().toLowerCase()
  return needle ? filterNodes(props.tree, needle) : props.tree
})
const counts = computed(() => {
  const levels = [0, 0, 0, 0]
  props.snapshot?.status.forEach((status) => { levels[status.level] = (levels[status.level] ?? 0) + 1 })
  return levels
})
const timestamp = computed(() => {
  if (!props.snapshot?.stamp) return ''
  return new Date(props.snapshot.stamp * 1000).toLocaleString()
})
const leafAlerts = computed(() => {
  const warnings: DiagnosticTreeNode[] = []
  const errors: DiagnosticTreeNode[] = []

  const visit = (nodes: DiagnosticTreeNode[]): void => {
    nodes.forEach((node) => {
      if (node.children.length) visit(node.children)
      else if (node.status?.level === 1) warnings.push(node)
      else if (node.status?.level === 2) errors.push(node)
    })
  }
  visit(props.tree)

  return { warnings, errors }
})

function filterNodes(nodes: DiagnosticTreeNode[], needle: string): DiagnosticTreeNode[] {
  return nodes.flatMap((node) => {
    const children = filterNodes(node.children, needle)
    const statusText = [
      node.path,
      node.status?.message,
      node.status?.hardwareId,
      ...(node.status?.values.flatMap((item) => [item.key, item.value]) ?? []),
    ].join(' ').toLowerCase()

    return statusText.includes(needle) || children.length
      ? [{ ...node, children }]
      : []
  })
}

function setExpansion(mode: ExpansionMode): void {
  expansionMode.value = mode
  expansionRevision.value += 1
  if (expandMenu.value) expandMenu.value.open = false
}
</script>

<template>
  <section aria-labelledby="diagnostics-title">
    <div class="section-heading">
      <div>
        <p class="eyebrow">Aggregated robot health</p>
        <h2 id="diagnostics-title">Diagnostics</h2>
      </div>
      <div class="heading-actions">
        <span v-if="timestamp" class="diagnostic-timestamp">Updated {{ timestamp }}</span>
        <details v-if="snapshot" ref="expandMenu" class="diagnostic-expand-menu">
          <summary class="secondary-button">Expand</summary>
          <div class="diagnostic-expand-options" role="menu" aria-label="Expand diagnostics">
            <button type="button" role="menuitem" @click="setExpansion('all')">All</button>
            <button type="button" role="menuitem" @click="setExpansion('warn-error')">
              Warn and err
            </button>
            <button type="button" role="menuitem" @click="setExpansion('error')">Err</button>
          </div>
        </details>
        <button
          v-if="snapshot"
          class="secondary-button"
          type="button"
          @click="setExpansion('collapse')"
        >Collapse</button>
      </div>
    </div>

    <div v-if="snapshot" class="diagnostic-summary" aria-label="Diagnostic status totals">
      <span class="diagnostic-summary--good"><strong>{{ counts[0] }}</strong> OK</span>
      <span class="diagnostic-summary--warning"><strong>{{ counts[1] }}</strong> warnings</span>
      <span class="diagnostic-summary--bad"><strong>{{ counts[2] }}</strong> errors</span>
      <span class="diagnostic-summary--neutral"><strong>{{ counts[3] }}</strong> stale</span>
    </div>

    <div
      v-if="leafAlerts.errors.length || leafAlerts.warnings.length"
      class="diagnostic-alert-summary"
      aria-label="Leaf diagnostic alerts"
    >
      <section
        v-if="leafAlerts.errors.length"
        class="diagnostic-alert-group diagnostic-alert-group--bad"
        aria-labelledby="diagnostic-errors-title"
      >
        <h3 id="diagnostic-errors-title">Errors <span>{{ leafAlerts.errors.length }}</span></h3>
        <ul>
          <li v-for="node in leafAlerts.errors" :key="node.path">
            <code>{{ node.path }}</code>
            <span>{{ node.status?.message || 'No message' }}</span>
          </li>
        </ul>
      </section>
      <section
        v-if="leafAlerts.warnings.length"
        class="diagnostic-alert-group diagnostic-alert-group--warning"
        aria-labelledby="diagnostic-warnings-title"
      >
        <h3 id="diagnostic-warnings-title">Warnings <span>{{ leafAlerts.warnings.length }}</span></h3>
        <ul>
          <li v-for="node in leafAlerts.warnings" :key="node.path">
            <code>{{ node.path }}</code>
            <span>{{ node.status?.message || 'No message' }}</span>
          </li>
        </ul>
      </section>
    </div>

    <label v-if="snapshot" class="search-field diagnostics-search">
      <span class="sr-only">Search diagnostics</span>
      <input v-model="query" type="search" placeholder="Search path, message, hardware, or value" />
    </label>

    <div v-if="!snapshot" class="state-message">Waiting for aggregated diagnostics…</div>
    <div v-else-if="!filteredTree.length" class="state-message">No matching diagnostics.</div>
    <div v-else class="diagnostic-tree-wrap">
      <ul class="diagnostic-tree" aria-label="Aggregated diagnostics tree">
        <DiagnosticsTreeNode
          v-for="node in filteredTree"
          :key="node.path"
          :node="node"
          :depth="0"
          :expansion-mode="expansionMode"
          :expansion-revision="expansionRevision"
        />
      </ul>
    </div>
  </section>
</template>
