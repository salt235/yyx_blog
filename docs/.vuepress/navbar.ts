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
      { text: '语言学习', link: '/notes/LanguageLearning/' },
      { text: '开发工具', link: '/notes/DevTools/' },
      { text: '科研与项目', link: '/notes/Projects/' },
      { text: '无人机入门', link: '/notes/UAV/' },
      { text: '机器学习与深度学习', link: '/notes/MachineLearning/' },
      { text: 'ROS学习', link: '/notes/ROS/' },
      { text: '文献阅读笔记', link: '/notes/Papers/' },
      { text: '杂项笔记&记录', link: '/notes/Misc/' }
    ],
  },
  { text: '友链', link: '/friends/' , icon: 'fa-solid:user-friends'}
])
