<script setup lang="ts">
import type { ThemeHero, ThemeHomeFeature } from 'vuepress-theme-plume/shared'
import VPButton from 'vuepress-theme-plume/components/VPButton.vue'
import VPLink from 'vuepress-theme-plume/components/VPLink.vue'
import { withBase } from 'vuepress/client'

defineOptions({ inheritAttrs: false })

defineProps<{
  banner: string
  hero: ThemeHero
  introduction: string
  featured: ThemeHomeFeature[]
}>()
</script>

<template>
  <section
    class="home-cover"
    :style="{ '--cover-image': `url(${withBase(banner)})` }"
    aria-labelledby="home-cover-title"
  >
    <div class="cover-content">
      <div class="cover-identity">
        <h1 id="home-cover-title" class="cover-name">{{ hero.name }}</h1>
        <p v-if="hero.tagline" class="cover-tagline">{{ hero.tagline }}</p>
        <p v-if="hero.text" class="cover-motto">{{ hero.text }}</p>
        <p class="cover-introduction">{{ introduction }}</p>
        <div class="cover-actions">
          <VPButton
            v-for="action in hero.actions"
            :key="action.link"
            tag="a"
            size="medium"
            :theme="action.theme"
            :text="action.text"
            :href="action.link"
            :target="action.target"
            :rel="action.rel"
          />
        </div>
      </div>

      <section class="cover-reading" aria-labelledby="home-featured-title">
        <h2 id="home-featured-title" class="cover-section-title">精选文章</h2>
        <ul class="cover-articles">
          <li v-for="article in featured" :key="article.link">
            <VPLink class="cover-article" :href="article.link" :no-icon="true">
              <div class="cover-article-copy">
                <h3 class="cover-article-title">{{ article.title }}</h3>
                <p class="cover-article-details">{{ article.details }}</p>
              </div>
              <span class="cover-article-arrow" aria-hidden="true">→</span>
            </VPLink>
          </li>
        </ul>
      </section>
    </div>
  </section>
</template>

<style scoped>
.home-cover {
  --vp-c-text-1: #f4f1f5;
  --vp-c-text-2: #d9d4de;
  --cover-accent: #a9dae5;
  --cover-bg: #252333;
  --cover-shade: 37 35 51;
  --cover-shade-left: 78%;
  --cover-shade-center: 32%;
  --cover-shade-right: 50%;
  --cover-reading-text: #fff;
  --cover-reading-accent: #c4e7ee;
  --cover-reading-shadow: 0 1px 3px rgb(23 21 33 / 95%), 0 0 1px #171521;
  --cover-divider: rgb(255 255 255 / 22%);
  --cover-button-hover: #d9d4de;
  --cover-button-border: rgb(217 212 222 / 35%);
  --cover-ui-font: var(--vp-font-family-base);

  position: relative;
  display: flex;
  align-items: center;
  min-height: calc(100vh - var(--vp-nav-height));
  min-height: calc(100svh - var(--vp-nav-height));
  padding: 40px 24px;
  color: var(--vp-c-text-1);
  background: var(--cover-bg);
  isolation: isolate;
}

[data-theme="light"] .home-cover {
  --vp-c-text-1: #292535;
  --vp-c-text-2: #494151;
  --cover-accent: #366b82;
  --cover-bg: #f1edf3;
  --cover-shade: 241 237 243;
  --cover-shade-left: 70%;
  --cover-shade-center: 44%;
  --cover-shade-right: 58%;
  --cover-reading-text: #292535;
  --cover-reading-accent: #366b82;
  --cover-reading-shadow: none;
  --cover-divider: rgb(41 37 53 / 22%);
  --cover-button-hover: #28546b;
  --cover-button-border: rgb(41 37 53 / 20%);
}

.home-cover::before,
.home-cover::after {
  position: absolute;
  inset: 0;
  content: "";
  z-index: -1;
  pointer-events: none;
}

.home-cover::before {
  background: var(--cover-image) 65% top / auto 720px no-repeat;
}

.home-cover::after {
  /* 手机上固定插画的构图高度，再融入底色，避免长页面把人物无限放大。 */
  background: linear-gradient(180deg, rgb(var(--cover-shade) / 65%), rgb(var(--cover-shade) / 78%) 440px, var(--cover-bg) 720px);
}

.cover-content {
  position: relative;
  display: grid;
  grid-template-columns: minmax(0, 1fr);
  gap: 32px;
  width: 100%;
  max-width: 1200px;
  margin: 0 auto;
}

.cover-name {
  margin: 0;
  font-size: clamp(52px, 7vw, 92px);
  font-weight: 600;
  line-height: 1.1;
}

.cover-tagline {
  margin: 12px 0 0;
  font-size: clamp(20px, 2.2vw, 28px);
  line-height: 1.5;
}

.cover-motto {
  margin: 8px 0 0;
  color: var(--vp-c-text-2);
  font-family: var(--cover-ui-font);
  font-size: 12px;
  letter-spacing: 0.04em;
  line-height: 1.8;
}

.cover-actions {
  --vp-button-brand-text: var(--cover-bg);
  --vp-button-brand-bg: var(--cover-accent);
  --vp-button-brand-hover-text: var(--cover-bg);
  --vp-button-brand-hover-bg: var(--cover-button-hover);
  --vp-button-brand-active-text: var(--cover-bg);
  --vp-button-brand-active-bg: var(--cover-accent);
  --vp-button-alt-text: var(--vp-c-text-1);
  --vp-button-alt-bg: rgb(var(--cover-shade) / 75%);
  --vp-button-alt-border: var(--cover-button-border);
  --vp-button-alt-hover-text: var(--vp-c-text-1);
  --vp-button-alt-hover-bg: var(--cover-bg);
  --vp-button-alt-active-text: var(--vp-c-text-1);
  --vp-button-alt-active-bg: var(--cover-bg);

  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  margin-top: 24px;
  font-family: var(--vp-font-family-base);
}

.cover-actions :deep(.vp-button) {
  font-weight: 400;
}

.cover-actions :deep(.vp-button + .vp-button) {
  margin-left: 0;
}

.cover-actions :deep(.vp-button:focus-visible) {
  outline: 2px solid var(--cover-accent);
  outline-offset: 4px;
}

.cover-reading {
  --vp-c-text-1: var(--cover-reading-text);
  --vp-c-text-2: var(--cover-reading-text);
  --cover-accent: var(--cover-reading-accent);

  justify-self: end;
  width: 100%;
  max-width: 480px;
  min-width: 0;
  padding: 12px 0;
  color: var(--vp-c-text-1);
  font-family: var(--vp-font-family-base);
  /* 只在字形边缘补深色阴影，让插画保持通透。 */
  text-shadow: var(--cover-reading-shadow);
}

.cover-introduction {
  max-width: 32em;
  margin: 24px 0 0;
  font-size: 16px;
  line-height: 1.9;
}

.cover-section-title {
  margin: 0 0 12px;
  color: var(--vp-c-brand-1);
  font-size: 24px;
  font-weight: 600;
  line-height: 1.4;
}

.cover-articles {
  padding: 0;
  margin: 0;
  list-style: none;
}

.cover-articles li + li {
  border-top: 1px solid var(--cover-divider);
}

.cover-article {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 20px 0;
  color: var(--vp-c-text-1);
  border-radius: 4px;
}

.cover-article-copy {
  flex: 1;
  min-width: 0;
}

.cover-article-title {
  margin: 0;
  font-size: 20px;
  font-weight: 600;
  line-height: 1.5;
}

.cover-article-details {
  margin: 8px 0 0;
  color: var(--vp-c-text-2);
  font-size: 16px;
  line-height: 1.8;
}

.cover-article-arrow {
  color: var(--cover-accent);
  font-size: 20px;
  transition: transform 180ms ease;
}

.cover-article:hover .cover-article-title {
  color: var(--cover-accent);
}

.cover-article:hover .cover-article-arrow {
  transform: translateX(3px);
  opacity: 1;
}

.cover-article:focus-visible {
  outline: 2px solid var(--cover-accent);
  outline-offset: 4px;
}

@media (min-width: 640px) {
  .home-cover {
    padding-top: 56px;
    padding-bottom: 56px;
    padding-right: 48px;
    padding-left: 48px;
  }

}

@media (max-width: 959px) {
  .cover-reading {
    max-width: none;
  }
}

@media (min-width: 960px) {
  .home-cover::before {
    background-size: cover;
    background-position: center;
  }

  .home-cover::after {
    background: linear-gradient(90deg, rgb(var(--cover-shade) / var(--cover-shade-left)), rgb(var(--cover-shade) / var(--cover-shade-center)) 56%, rgb(var(--cover-shade) / var(--cover-shade-right)));
  }

  .cover-content {
    grid-template-columns: minmax(0, 0.9fr) minmax(0, 1.1fr);
    gap: clamp(40px, 5vw, 80px);
    align-items: center;
  }
}

@media (prefers-reduced-motion: reduce) {
  .cover-article-arrow {
    transition: none;
  }
}
</style>
