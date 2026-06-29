<script setup lang="ts">
import { computed, ref } from 'vue'
import { RouterLink } from 'vue-router'
import { usePostsData } from 'vuepress-theme-plume/client'

const postsData = usePostsData()
const posts = computed(() =>
  (postsData.value['/running/'] ?? []).filter(post => post.path !== '/running/'),
)
const pageSize = 3
const currentPage = ref(1)
const totalPages = computed(() => Math.max(1, Math.ceil(posts.value.length / pageSize)))
const visiblePosts = computed(() => {
  const start = (currentPage.value - 1) * pageSize
  return posts.value.slice(start, start + pageSize)
})

function formatDate(value?: string) {
  if (!value) return '日期未记录'
  return value.slice(0, 10).replaceAll('/', '.')
}

function goToPage(page: number) {
  currentPage.value = Math.min(totalPages.value, Math.max(1, page))
}
</script>

<template>
  <main class="running-journal">
    <section class="running-hero">
      <div class="running-hero__copy">
        <p class="running-kicker">RUNNING LOG · KEEP MOVING</p>
        <h1>跑步小记</h1>
        <p class="running-lead">
          记录训练、比赛、伤病，也记录一次次重新出发。
          这里不追求标准答案，只留下身体和时间共同写下的轨迹。
        </p>
        <div class="running-stats" aria-label="跑步小记统计">
          <span><strong>{{ posts.length }}</strong> 篇记录</span>
        </div>
      </div>
    </section>

    <section class="running-log">
      <header class="running-log__header">
        <div>
          <h2>沿途记录</h2>
          <p class="running-log__order">按时间倒序 · 最新记录在前</p>
        </div>
      </header>

      <div v-if="posts.length" class="running-timeline-wrap">
        <div class="running-timeline">
          <article v-for="post in visiblePosts" :key="post.path" class="running-entry">
            <RouterLink class="running-entry__card" :to="post.path">
              <div class="running-entry__meta">
                <time>{{ formatDate(post.createTime) }}</time>
                <span v-if="post.readingTime">{{ Math.max(1, Math.round(post.readingTime.minutes)) }} MIN READ</span>
              </div>
              <div class="running-entry__main">
                <h3>{{ post.title }}</h3>
                <span class="running-entry__arrow">继续阅读 →</span>
              </div>
            </RouterLink>
          </article>
        </div>
        <nav v-if="totalPages > 1" class="running-pagination" aria-label="跑步小记分页">
          <button :disabled="currentPage === 1" @click="goToPage(currentPage - 1)">← 上一页</button>
          <button
            v-for="page in totalPages"
            :key="page"
            class="running-pagination__number"
            :class="{ active: page === currentPage }"
            :aria-current="page === currentPage ? 'page' : undefined"
            @click="goToPage(page)"
          >
            {{ page }}
          </button>
          <button :disabled="currentPage === totalPages" @click="goToPage(currentPage + 1)">下一页 →</button>
        </nav>
      </div>

      <div v-else class="running-empty">第一篇记录，正在下一公里等你。</div>
    </section>
  </main>
</template>
