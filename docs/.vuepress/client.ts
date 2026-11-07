import { defineClientConfig } from 'vuepress/client'
import { defineAsyncComponent } from 'vue'
// import RepoCard from 'vuepress-theme-plume/features/RepoCard.vue'
// import NpmBadge from 'vuepress-theme-plume/features/NpmBadge.vue'
// import NpmBadgeGroup from 'vuepress-theme-plume/features/NpmBadgeGroup.vue'
// import Swiper from 'vuepress-theme-plume/features/Swiper.vue'

// import CustomComponent from './theme/components/Custom.vue'

import './theme/styles/custom.css' // your custom styles
import HomeCover from './theme/components/HomeCover.vue'

export default defineClientConfig({
  enhance({ app }) {
    app.component('HomeCover', HomeCover)
    app.component('RunningDashboard', defineAsyncComponent(() => import('./theme/components/running/RunningDashboard.vue')))
    // built-in components
    // app.component('RepoCard', RepoCard)
    // app.component('NpmBadge', NpmBadge)
    // app.component('NpmBadgeGroup', NpmBadgeGroup)
    // app.component('Swiper', Swiper) // you should install `swiper`

    // your custom components
    // app.component('CustomComponent', CustomComponent)
  },
})
