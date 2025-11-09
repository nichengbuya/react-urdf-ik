import { useEffect, useRef, useState, useCallback, useMemo, JSX } from "react";
import { createPortal } from "react-dom";
import URDFViewer, { JointMeta, URDFViewerHandle } from "./URDFViewer";

type Deg = number;

function clamp(v: number, lo: number, hi: number) { return Math.max(lo, Math.min(hi, v)); }
function radToDeg(r: number) { return (r * 180) / Math.PI; }
function escapeReg(s: string) { return s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"); }

type RobotsManifest = { generatedAt?: string; urdfs: string[] };

// ===== 统一输入样式（类 antd） =====
const INPUT_STYLE: React.CSSProperties = {
  width: "100%",
  boxSizing: "border-box",
  padding: "8px 10px",
  border: "1px solid #d9d9d9",
  borderRadius: 6,
  outline: "none",
  background: "#fff",
  fontSize: 14,
  lineHeight: "22px",
  fontFamily: "system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, 'Apple Color Emoji','Segoe UI Emoji', 'Segoe UI Symbol'",
};

const NUMBER_INPUT_STYLE: React.CSSProperties = {
  ...INPUT_STYLE,
  width: 140,
};

// ====== 高亮工具 ======
function highlightTokens(text: string, query: string) {
  const q = query.trim().toLowerCase();
  if (!q) return text;
  const tokens = q.split(/\s+/).filter(Boolean);
  if (!tokens.length) return text;

  let out: Array<string | JSX.Element> = [text];
  tokens.forEach((tok) => {
    const next: Array<string | JSX.Element> = [];
    out.forEach((piece) => {
      if (typeof piece !== "string") { next.push(piece); return; }
      const parts = piece.split(new RegExp(`(${escapeReg(tok)})`, "ig"));
      parts.forEach((p, i) => {
        if (i % 2 === 1) next.push(<mark key={p + Math.random()}>{p}</mark>);
        else next.push(p);
      });
    });
    out = next;
  });
  return <>{out}</>;
}

// ====== 用 canvas 精确测量文本宽度（用于下拉自适应宽度）======
function measureTextWidth(texts: string[], font: string): number {
  if (typeof document === "undefined") return 0;
  const canvas = document.createElement("canvas");
  const ctx = canvas.getContext("2d");
  if (!ctx) return 0;
  ctx.font = font;
  let max = 0;
  for (const t of texts) {
    const m = ctx.measureText(t);
    const w = m.width;
    if (w > max) max = w;
  }
  return Math.ceil(max);
}

// ====== 计算下拉面板 fixed 定位，并让宽度足够显示完整文本 ======
function useDropdownPositionAndWidth() {
  const [rect, setRect] = useState<{
    left: number; top: number; width: number; maxHeight: number; direction: "down" | "up";
  } | null>(null);

  const update = useCallback((args: {
    inputEl: HTMLInputElement | null;
    items: string[];
    font: string;        // e.g. "14px system-ui"
    paddingX: number;    // 面板左右内边距 + 行左右 padding
    minWidth?: number;   // 至少等于输入框宽
    maxWidthMargin?: number; // 距离视口左右留白
    idealMaxHeight?: number; // 理想最大高度
  }) => {
    const {
      inputEl, items, font, paddingX,
      minWidth = 0, maxWidthMargin = 8, idealMaxHeight = 320,
    } = args;
    if (!inputEl) { setRect(null); return; }

    const r = inputEl.getBoundingClientRect();
    const vw = window.innerWidth;
    const vh = window.innerHeight;

    // 计算最佳宽度：取(最长文本宽度 + paddingX)，再不小于输入框宽
    const longest = measureTextWidth(items.length ? items : [inputEl.value || ""], font);
    const rawWidth = Math.max(minWidth, longest + paddingX);

    // 视口防溢出
    const maxAvailWidth = Math.max(240, vw - maxWidthMargin * 2);
    const width = Math.min(rawWidth, maxAvailWidth);

    const margin = 6;
    const spaceBelow = vh - r.bottom - margin;
    const spaceAbove = r.top - margin;

    const panelIdealH = idealMaxHeight;
    let direction: "down" | "up";
    let top: number;
    let maxHeight: number;

    if (spaceBelow >= Math.min(panelIdealH, vh * 0.6) || spaceBelow >= spaceAbove) {
      direction = "down";
      top = r.bottom + margin;
      maxHeight = Math.max(140, Math.min(panelIdealH, spaceBelow));
    } else {
      direction = "up";
      const height = Math.max(140, Math.min(panelIdealH, spaceAbove));
      top = Math.max(margin, r.top - margin - height);
      maxHeight = height;
    }

    // 让左侧对齐输入框，若右侧会溢出则左移
    let left = r.left;
    const overflowRight = left + width + maxWidthMargin - vw;
    if (overflowRight > 0) {
      left = Math.max(maxWidthMargin, left - overflowRight);
    }
    // 也避免左边溢出
    if (left < maxWidthMargin) left = maxWidthMargin;

    setRect({ left, top, width, maxHeight, direction });
  }, []);

  return { rect, update, clear: () => setRect(null) };
}

// ====== Portal：把下拉固定到 body 上，避免裁切 ======
function DropdownPortal({ children, rect }: { children: React.ReactNode; rect: { left: number; top: number; width: number; maxHeight: number } }) {
  if (!rect) return null as any;
  const style: React.CSSProperties = {
    position: "fixed",
    left: rect.left,
    top: rect.top,
    width: rect.width,
    maxHeight: rect.maxHeight,
    overflowY: "auto",
    background: "#fff",
    border: "1px solid #d9d9d9",
    boxShadow: "0 6px 16px rgba(0,0,0,0.08)",
    borderRadius: 6,
    zIndex: 10000,
    padding: 4,
  };
  return createPortal(<div style={style}>{children}</div>, document.body);
}

export default function App() {
  const [urdfList, setUrdfList] = useState<string[]>([]);
  const [loadingList, setLoadingList] = useState(false);
  const [listError, setListError] = useState<string | null>(null);

  // ▼ 搜索关键词
  const [urdfQuery, setUrdfQuery] = useState("");

  const [urdfUrl, setUrdfUrl] = useState(
    "/robots/ros-industrial/xacro_generated/kuka/kuka_kr10_support/urdf/kr10r900_2.urdf"
  );
  const [baseLink, setBaseLink] = useState("base_link");
  const [eeLink, setEeLink] = useState("tool0");

  const [jointsMeta, setJointsMeta] = useState<JointMeta[]>([]);
  const [qDeg, setQDeg] = useState<Deg[]>([]);
  const viewerRef = useRef<URDFViewerHandle>(null);

  const hasMeta = jointsMeta.length > 0;
  const initialPoseDegPreset: Deg[] = [0, -90, 100, 0, 60, 100];

  // ▼ 下拉交互
  const [comboOpen, setComboOpen] = useState(false);
  const [activeIndex, setActiveIndex] = useState(0);
  const inputRef = useRef<HTMLInputElement>(null);
  const comboWrapRef = useRef<HTMLDivElement>(null);
  const panelMouseDownRef = useRef(false); // 防止点击面板导致的 blur 先关

  // 点击外部关闭（含 portal）
  useEffect(() => {
    const onDocMouseDown = (e: MouseEvent) => {
      const path = e.composedPath() as EventTarget[];
      const fromPanel = path.some((n) => (n as HTMLElement)?.dataset?.dropPanel === "1");
      const fromWrap = comboWrapRef.current?.contains(e.target as Node) ?? false;
      if (!fromPanel && !fromWrap) setComboOpen(false);
    };
    document.addEventListener("mousedown", onDocMouseDown);
    return () => document.removeEventListener("mousedown", onDocMouseDown);
  }, []);

  // 拉清单
  useEffect(() => {
    let aborted = false;
    (async () => {
      setLoadingList(true);
      setListError(null);
      try {
        const res = await fetch("/robots/manifest.json", { cache: "no-store" });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data: RobotsManifest = await res.json();
        if (!aborted) setUrdfList(Array.isArray(data.urdfs) ? data.urdfs : []);
      } catch (e: any) {
        if (!aborted) {
          setListError(e?.message || String(e));
          setUrdfList([]);
        }
      } finally {
        if (!aborted) setLoadingList(false);
      }
    })();
    return () => { aborted = true; };
  }, []);

  const onJointsReady = useCallback((meta: JointMeta[]) => {
    setJointsMeta(meta);
    const pose: Deg[] = meta.map((j, i) => {
      const v = initialPoseDegPreset[i] ?? 0;
      const lo = (j.lower ?? (j.unit === "deg" ? -180 : -Infinity));
      const hi = (j.upper ?? (j.unit === "deg" ? 180 : Infinity));
      return clamp(v, lo, hi);
    });
    setQDeg(pose);
  }, []);

  // ★ IK 回传：阈值去抖（角 0.05°，直线 0.0005m）
  const onQChangeFromViewer = useCallback((qInternal: number[]) => {
    setQDeg((prev) => {
      const nextDeg = qInternal.map((v, i) => {
        const unit = jointsMeta[i]?.unit ?? "deg";
        return unit === "deg" ? radToDeg(v) : v;
      });
      if (prev.length !== nextDeg.length) return nextDeg;

      let changed = false;
      for (let i = 0; i < nextDeg.length; i++) {
        const unit = jointsMeta[i]?.unit ?? "deg";
        const tol = unit === "deg" ? 0.05 : 0.0005;
        if (Math.abs((nextDeg[i] ?? 0) - (prev[i] ?? 0)) > tol) { changed = true; break; }
      }
      return changed ? nextDeg : prev;
    });
  }, [jointsMeta]);

  const onJointSlider = (idx: number, valDeg: Deg) => {
    setQDeg((prev) => {
      const next = prev.slice();
      next[idx] = valDeg;
      return next;
    });
  };

  const resetQ = () => {
    if (!hasMeta) return;
    const zeros = jointsMeta.map((j) => {
      const lo = j.lower ?? (j.unit === "deg" ? -180 : -Infinity);
      const hi = j.upper ?? (j.unit === "deg" ? 180 : Infinity);
      return clamp(0, lo, hi);
    });
    setQDeg(zeros);
  };

  // ▼ 搜索过滤（大小写不敏感；空格分词匹配）
  const filteredUrdfs = useMemo(() => {
    const q = urdfQuery.trim().toLowerCase();
    if (!q) return urdfList;
    const tokens = q.split(/\s+/).filter(Boolean);
    return urdfList.filter(u => {
      const s = u.toLowerCase();
      return tokens.every(t => s.includes(t));
    });
  }, [urdfList, urdfQuery]);

  // ▼ 选择某个 URDF
  const pickUrdf = (u: string) => {
    setUrdfUrl(u);
    setUrdfQuery(u); // 让输入框也展示完整路径，方便确认
  };

  // ▼ 驱动“永远可见 + 自适应宽度”的定位
  const { rect, update, clear } = useDropdownPositionAndWidth();
  useEffect(() => {
    const el = inputRef.current;
    if (!comboOpen) { clear(); return; }

    // 面板左右 padding + 每行左右 padding（用于文本真实宽度之外的 UI 内边距）
    const paddingX = 16 /*panel*/ + 20 /*row*/;

    // 使用与 INPUT_STYLE 对应的字体
    const font = "14px system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial";

    // 更新定位与宽度
    update({
      inputEl: el,
      items: filteredUrdfs,
      font,
      paddingX,
      minWidth: el ? el.getBoundingClientRect().width : 0,
      maxWidthMargin: 8,
      idealMaxHeight: 360,
    });

    const onWin = () => update({
      inputEl: el,
      items: filteredUrdfs,
      font,
      paddingX,
      minWidth: el ? el.getBoundingClientRect().width : 0,
      maxWidthMargin: 8,
      idealMaxHeight: 360,
    });

    window.addEventListener("resize", onWin);
    window.addEventListener("scroll", onWin, true);
    const i = setInterval(onWin, 160);
    return () => {
      window.removeEventListener("resize", onWin);
      window.removeEventListener("scroll", onWin, true);
      clearInterval(i);
    };
  }, [comboOpen, urdfQuery, filteredUrdfs, update, clear]);

  return (
    <div className="app" style={{ display: "flex", height: "100vh" }}>
      <div style={{ width: 430, padding: 12, borderRight: "1px solid #eee", background: "#fff" }}>
        <h3 style={{ marginTop: 4 }}>URDF · FK/IK + Joint UI</h3>

        {/* 搜索 + 下拉（自适应宽度 + 永远可见） */}
        <div style={{ marginBottom: 10 }}>
          <div>搜索 URDF（支持路径关键词、空格分词）</div>
          <div ref={comboWrapRef} style={{ position: "relative" }}>
            <input
              ref={inputRef}
              value={urdfQuery}
              onChange={(e) => {
                setUrdfQuery(e.target.value);
                setComboOpen(true);
                setActiveIndex(0);
              }}
              onFocus={() => setComboOpen(true)}
              onKeyDown={(e) => {
                if (!comboOpen && (e.key === "ArrowDown" || e.key === "Enter")) {
                  setComboOpen(true);
                  return;
                }
                if (e.key === "ArrowDown") {
                  e.preventDefault();
                  setActiveIndex((i) => Math.min(i + 1, Math.max(0, filteredUrdfs.length - 1)));
                } else if (e.key === "ArrowUp") {
                  e.preventDefault();
                  setActiveIndex((i) => Math.max(i - 1, 0));
                } else if (e.key === "Enter") {
                  e.preventDefault();
                  const u = filteredUrdfs[activeIndex];
                  if (u) {
                    pickUrdf(u);
                    setComboOpen(false);
                  }
                } else if (e.key === "Escape") {
                  setComboOpen(false);
                }
              }}
              placeholder="例如：kuka kr10 urdf / abb irb2400"
              style={INPUT_STYLE}
              disabled={loadingList || !!listError}
              onBlur={() => {
                // 如果是点击了面板（onMouseDown 标记），就不在 blur 时关闭
                if (panelMouseDownRef.current) {
                  panelMouseDownRef.current = false;
                  inputRef.current?.focus(); // 保持焦点，体验更像 antd
                } else {
                  setComboOpen(false);
                }
              }}
            />

            {comboOpen && !loadingList && !listError && rect && (
              <DropdownPortal rect={rect}>
                <div data-drop-panel="1">
                  {filteredUrdfs.length === 0 ? (
                    <div style={{ padding: "10px 12px", color: "#999", whiteSpace: "nowrap" }}>无匹配结果</div>
                  ) : (
                    filteredUrdfs.slice(0, 1000).map((u, idx) => {
                      const isActive = idx === activeIndex;
                      return (
                        <div
                          key={u}
                          onMouseEnter={() => setActiveIndex(idx)}
                          onMouseDown={(e) => {
                            // 防止 blur 把面板关掉
                            panelMouseDownRef.current = true;
                            e.preventDefault();
                            pickUrdf(u);
                            setComboOpen(false);
                          }}
                          title={u}
                          style={{
                            cursor: "pointer",
                            padding: "6px 10px",
                            borderRadius: 4,
                            background: isActive ? "#e6f4ff" : "transparent",
                            border: isActive ? "1px solid #91caff" : "1px solid transparent",
                            margin: "2px 0",
                            fontFamily: "ui-monospace,monospace",
                            fontSize: 12,
                            whiteSpace: "nowrap",
                          }}
                        >
                          {highlightTokens(u, urdfQuery)}
                        </div>
                      );
                    })
                  )}
                </div>
              </DropdownPortal>
            )}
          </div>

          <div style={{ fontSize: 12, color: "#888", marginTop: 6 }}>
            你也可以在下方输入框中<strong>手动填写 URL</strong>。
          </div>
        </div>

        {/* 手动 URL、base、EE */}
        <div style={{ marginBottom: 8 }}>
          <div>URDF URL</div>
          <input
            value={urdfUrl}
            onChange={(e) => setUrdfUrl(e.target.value)}
            style={INPUT_STYLE}
            placeholder="/robots/xxx/.../file.urdf"
          />
        </div>

        <div style={{ marginBottom: 8 }}>
          <div>Base link</div>
          <input value={baseLink} onChange={(e) => setBaseLink(e.target.value)} style={INPUT_STYLE} />
        </div>
        <div style={{ marginBottom: 8 }}>
          <div>EE link</div>
          <input value={eeLink} onChange={(e) => setEeLink(e.target.value)} style={INPUT_STYLE} />
        </div>

        <div style={{ display: "flex", gap: 8, alignItems: "center", margin: "12px 0 8px" }}>
          <button onClick={resetQ} disabled={!hasMeta}>关节清零</button>
          <button onClick={() => viewerRef.current?.recenterHandle()} disabled={!hasMeta} title="将把手移动到当前 FK 末端位姿">
            重新摆放 handle
          </button>
          <span style={{ fontSize: 12, color: "#666" }}>（单位：° 或 m）</span>
        </div>

        <div style={{ maxHeight: "52vh", overflow: "auto", paddingRight: 4 }}>
          {jointsMeta.map((j, i) => {
            const unitLabel = j.unit === "deg" ? "°" : "m";
            const min = j.lower ?? (j.unit === "deg" ? -180 : -1);
            const max = j.upper ?? (j.unit === "deg" ?  180 :  1);
            const step = j.unit === "deg" ? 0.5 : 0.001;
            const val = clamp(qDeg[i] ?? 0, min, max);
            const finite = Number.isFinite(min) && Number.isFinite(max);

            return (
              <div key={j.name + i} style={{ border: "1px solid #eee", borderRadius: 8, padding: 8, marginBottom: 8, background: "#fafafa" }}>
                <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 6, fontFamily: "ui-monospace, monospace" }}>
                  <strong>{j.name}</strong>
                  <span style={{ color: "#666" }}>
                    [{Number.isFinite(min) ? min.toFixed(j.unit === "deg" ? 1 : 3) : "-∞"},
                     {Number.isFinite(max) ? max.toFixed(j.unit === "deg" ? 1 : 3) : "+∞"}]{unitLabel}
                    {"  "}类型: {j.type}
                  </span>
                </div>

                <input
                  type="range"
                  min={Number.isFinite(min) ? min : 0}
                  max={Number.isFinite(max) ? max : 0}
                  step={step}
                  value={Number.isFinite(val) ? val : 0}
                  onChange={(e) => onJointSlider(i, Number(e.target.value))}
                  style={{ width: "100%" }}
                  disabled={!finite}
                />

                <div style={{ display: "flex", gap: 8, marginTop: 6, alignItems: "center" }}>
                  <input
                    type="number"
                    value={Number.isFinite(val) ? val : 0}
                    step={step}
                    min={Number.isFinite(min) ? min : undefined}
                    max={Number.isFinite(max) ? max : undefined}
                    onChange={(e) => onJointSlider(i, clamp(Number(e.target.value), min, max))}
                    placeholder={j.unit === "deg" ? "角度(°)" : "位移(m)"}
                    style={NUMBER_INPUT_STYLE}
                  />
                  <span style={{ color: "#666" }}>单位: {unitLabel}</span>
                </div>
              </div>
            );
          })}
        </div>
      </div>

      <div style={{ position: "relative", flex: 1, background: "#0b0b0b" }}>
        <URDFViewer
          ref={viewerRef}
          urdfUrl={urdfUrl}
          baseLink={baseLink}
          eeLink={eeLink}
          qDeg={qDeg}
          onJointsReady={onJointsReady}
          onQChange={onQChangeFromViewer} // 仅内部更新时触发，避免死循环
        />
      </div>
    </div>
  );
}
