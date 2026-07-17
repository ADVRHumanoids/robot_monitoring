<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import type { DiagnosticTreeNode } from '../types'
import StatusBadge from './StatusBadge.vue'

defineOptions({ name: 'DiagnosticsTreeNode' })

type ExpansionMode = 'all' | 'warn-error' | 'error' | 'collapse'

const props = defineProps<{
  node: DiagnosticTreeNode
  depth: number
  expansionMode: ExpansionMode
  expansionRevision: number
}>()

const hasDetails = computed(() =>
  Boolean(props.node.children.length || props.node.status?.hardwareId || props.node.status?.values.length),
)
const defaultOpen = computed(() => props.depth < 2 || props.node.level > 0)
const levelLabel = computed(() => ['OK', 'WARN', 'ERROR', 'STALE'][props.node.level] ?? 'UNKNOWN')
const isOpen = ref(
  props.expansionRevision
    ? shouldOpen(props.expansionMode)
    : defaultOpen.value,
)

watch(
  () => props.expansionRevision,
  () => { isOpen.value = shouldOpen(props.expansionMode) },
)

function shouldOpen(mode: ExpansionMode): boolean {
  if (mode === 'all') return true
  if (mode === 'collapse') return props.depth < 2
  const levels = mode === 'warn-error' ? new Set([1, 2]) : new Set([2])
  return branchContainsLevel(props.node, levels)
}

function branchContainsLevel(node: DiagnosticTreeNode, levels: ReadonlySet<number>): boolean {
  return Boolean(
    (node.status && levels.has(node.status.level))
    || node.children.some((child) => branchContainsLevel(child, levels)),
  )
}

function onToggle(event: Event): void {
  isOpen.value = (event.currentTarget as HTMLDetailsElement).open
}
</script>

<template>
  <li class="diagnostic-tree-node" :data-path="node.path">
    <details v-if="hasDetails" :open="isOpen" @toggle="onToggle">
      <summary class="diagnostic-node-row">
        <span class="diagnostic-node-name">{{ node.label }}</span>
        <span
          v-if="node.status?.message"
          class="diagnostic-node-message"
          :title="node.status.message"
        >{{ node.status.message }}</span>
        <StatusBadge :status="levelLabel" />
      </summary>

      <div v-if="node.status?.hardwareId || node.status?.values.length" class="diagnostic-node-details">
        <p v-if="node.status.hardwareId"><strong>Hardware:</strong> {{ node.status.hardwareId }}</p>
        <dl v-if="node.status.values.length" class="diagnostic-values">
          <div v-for="(item, index) in node.status.values" :key="`${index}:${item.key}`">
            <dt>{{ item.key }}</dt><dd>{{ item.value }}</dd>
          </div>
        </dl>
      </div>

      <ul v-if="node.children.length" class="diagnostic-tree-children">
        <DiagnosticsTreeNode
          v-for="child in node.children"
          :key="child.path"
          :node="child"
          :depth="depth + 1"
          :expansion-mode="expansionMode"
          :expansion-revision="expansionRevision"
        />
      </ul>
    </details>

    <div v-else class="diagnostic-node-row diagnostic-node-row--leaf">
      <span class="diagnostic-node-name">{{ node.label }}</span>
      <span
        v-if="node.status?.message"
        class="diagnostic-node-message"
        :title="node.status.message"
      >{{ node.status.message }}</span>
      <StatusBadge :status="levelLabel" />
    </div>
  </li>
</template>
