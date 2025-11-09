// Node >=18
import { promises as fs } from "fs";
import path from "path";
import { fileURLToPath } from "url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const PUBLIC_DIR = path.resolve(process.cwd(), "public");
const ROBOTS_DIR = path.join(PUBLIC_DIR, "robots");
const MANIFEST = path.join(ROBOTS_DIR, "manifest.json");

// 递归收集 .urdf 文件
async function walk(dir) {
  const out = [];
  const ents = await fs.readdir(dir, { withFileTypes: true });
  for (const e of ents) {
    const full = path.join(dir, e.name);
    if (e.isDirectory()) {
      out.push(...(await walk(full)));
    } else if (e.isFile() && e.name.toLowerCase().endsWith(".urdf")) {
      out.push(full);
    }
  }
  return out;
}

async function run() {
  try {
    await fs.mkdir(ROBOTS_DIR, { recursive: true });

    // robots 目录可能为空，也要写个空清单
    let files = [];
    try {
      files = await walk(ROBOTS_DIR);
    } catch (e) {
      // 没有 robots 目录时保底为空
      files = [];
    }

    // 转为以 /robots/... 开头的 URL
    const urdfs = files
      .map((abs) => abs.replace(PUBLIC_DIR, "").replace(/\\/g, "/"))
      .sort();

    const data = { generatedAt: new Date().toISOString(), urdfs };
    await fs.writeFile(MANIFEST, JSON.stringify(data, null, 2), "utf-8");

    console.log(`[robots] manifest generated: ${MANIFEST} (${urdfs.length} files)`);
  } catch (err) {
    console.error("[robots] manifest generation failed:", err);
    process.exitCode = 1;
  }
}

run();
