/**
 * @see https://theme-plume.vuejs.press/config/navigation/ 查看文档了解配置详情
 *
 * Navbar 配置文件，它在 `.vuepress/plume.config.ts` 中被导入。
 */

import { defineNavbarConfig } from 'vuepress-theme-plume'

export default defineNavbarConfig([
  { text: '首页', link: '/' , icon: 'material-symbols:home-rounded'},
  { text: '博客', link: '/blog/' , icon: 'material-symbols:bookmark-manager-rounded'},
  { 
    text: '笔记', icon: 'material-symbols:book-4-rounded',
    items: [
      { text: 'Java', link: '/notes/Java/' },
      { text: 'DL', link: '/notes/Lee_DL/' },
      { text: 'ROS', link: '/notes/ROS/' },
      { text: 'VLN', link: '/notes/VLN/' },
    ],
  },
  { text: '友链', link: '/friends/' , icon: 'fa-solid:user-friends'},
])