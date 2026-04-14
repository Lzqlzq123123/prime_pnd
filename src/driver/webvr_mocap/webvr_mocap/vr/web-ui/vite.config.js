import { defineConfig } from 'vite';
import { viteStaticCopy } from 'vite-plugin-static-copy';

export default defineConfig({
    build: {
        outDir: 'dist',          // 输出目录
        minify: 'terser',        // 用 Terser 压缩混淆
        terserOptions: {         // 混淆规则（按需调整）
            compress: {
                drop_console: true,  // 移除 console.log
                drop_debugger: true, // 移除 debugger
            },
            mangle: {
                toplevel: true,            // 混淆顶层变量/函数（核心！默认仅混淆局部）
                keep_classnames: false,    // 不保留类名（类名也会被混淆）
                keep_fnames: false,
                safari10: false,           // 无需兼容 Safari 10（兼容则降低混淆）
                reserved: ['enterVR'],
                // 混淆对象属性（关键：让 obj.xxx → obj._0x123）
                // properties: {
                //     regex: /^[a-zA-Z_$][a-zA-Z0-9_$]*$/, // 匹配所有合法属性名
                //     keep_quoted: true,     // 即使属性加引号也混淆（如 obj['name'] → obj._0xabc）
                //     reserved: ['enterVR'],           // 无需保留任何属性名（按需添加需保留的属性）
                // },
            },
            format: {
                comments: false,     // 移除所有注释
            },
        },
        rollupOptions: {
            input: {
                index: './index.html',
            },
            output: {
                manualChunks: () => 'index.js', // 所有 JS 合并为单个文件
                assetFileNames: '[name][extname]', // 静态资源保留原文件名
            },
        },
        assetsInclude: ['**/interface.js'], // 第三方 min.js 直接复制
    },
    plugins: [
        viteStaticCopy({
            targets: [
                { src: './aframe.min.js', dest: './' },
                { src: './interface.js', dest: './' },
                { src: './favicon.ico', dest: './' },
                { src: './font/Roboto-msdf.json', dest: './font/' },
                { src: './font/Roboto-msdf.png', dest: './font/' }
            ]
        })
    ]
});