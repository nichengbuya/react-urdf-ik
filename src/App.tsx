import {
  useEffect,
  useRef,
  useState,
  useCallback,
  useMemo,
  type CSSProperties,
  JSX,
} from "react";
import { Input, InputNumber, Slider, Button, Typography, Card, Space, Alert, Spin, Divider, Select } from "antd";
import URDFViewer, { JointMeta, URDFViewerHandle } from "./URDFViewer";

const { Title, Text } = Typography;

type Deg = number;

function clamp(v: number, lo: number, hi: number) {
  return Math.max(lo, Math.min(hi, v));
}
function radToDeg(r: number) {
  return (r * 180) / Math.PI;
}

type RobotsManifest = { generatedAt?: string; urdfs: string[] };



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

  const [handlePos, setHandlePos] = useState<[number, number, number]>([0, 0, 0]);
  const [handleEulerDeg, setHandleEulerDeg] = useState<[number, number, number]>([0, 0, 0]);
  const initialPoseDegPreset: Deg[] = [0, -90, 100, 0, 60, 100];

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
    return () => {
      aborted = true;
    };
  }, []);

  const onJointsReady = useCallback((meta: JointMeta[]) => {
    setJointsMeta(meta);
    const pose: Deg[] = meta.map((j, i) => {
      const v = initialPoseDegPreset[i] ?? 0;
      const lo = j.lower ?? (j.unit === "deg" ? -180 : -Infinity);
      const hi = j.upper ?? (j.unit === "deg" ? 180 : Infinity);
      return clamp(v, lo, hi);
    });
    setQDeg(pose);
  }, []);

  // ★ IK 回传：阈值去抖（角 0.05°，直线 0.0005m）
  const onQChangeFromViewer = useCallback(
    (qInternal: number[]) => {
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
          if (Math.abs((nextDeg[i] ?? 0) - (prev[i] ?? 0)) > tol) {
            changed = true;
            break;
          }
        }
        return changed ? nextDeg : prev;
      });
    },
    [jointsMeta]
  );

  const onJointSlider = (idx: number, valDeg: Deg) => {
    setQDeg((prev) => {
      const next = prev.slice();
      next[idx] = valDeg;
      return next;
    });
  };

  // ▼ 搜索过滤（大小写不敏感；空格分词匹配）
  const filteredUrdfs = useMemo(() => {
    const q = urdfQuery.trim().toLowerCase();
    if (!q) return urdfList;
    const tokens = q.split(/\s+/).filter(Boolean);
    return urdfList.filter((u) => {
      const s = u.toLowerCase();
      return tokens.every((t) => s.includes(t));
    });
  }, [urdfList, urdfQuery]);

  const urdfOptions = useMemo(
    () =>
      filteredUrdfs.slice(0, 1000).map((u) => ({
        value: u,
        label: u
      })),
    [filteredUrdfs]
  );

  // ▼ 选择某个 URDF
  const pickUrdf = (u: string) => {
    setUrdfUrl(u);
    setUrdfQuery(u);
  };

  return (
<div className="app" style={{ display: "flex", height: "100vh" }}>
  {/* 左侧控制区：改成 flex column，操作区占满剩余高度 */}
  <div
    style={{
      width: 430,
      padding: 8,
      borderRight: "1px solid #eee",
      background: "#fff",
      display: "flex",
      flexDirection: "column",
      boxSizing: "border-box",
    }}
  >
    {/* 上半部分：标题 + 搜索 + Base/EE，尽量紧凑 */}
    <div style={{ paddingBottom: 4 }}>
      <Title level={5} style={{ margin: 0, marginBottom: 6 }}>
        URDF · FK/IK + Joint UI
      </Title>

      {/* 搜索 + 下拉（Antd AutoComplete） */}
      <div style={{ marginBottom: 4 }}>
        <Text style={{ fontSize: 12 }}>搜索 URDF（支持路径关键词、空格分词）</Text>
        <div style={{ marginTop: 4 }}>
              <Select
                showSearch
                value={urdfQuery}
                onSearch={(val) => setUrdfQuery(val)}
                onChange={(val) => {
                  const v = (val as string) || "";
                  setUrdfQuery(v);
                  if (v) {
                    pickUrdf(v);   // 有实际值时才选中
                  } else {
                    // 如果你希望清空时也清 url，这里可以顺便 reset
                    // setUrdfUrl("");
                  }
                }}
                options={urdfOptions}
                placeholder="例如：kuka kr10 urdf / abb irb2400"
                allowClear
                popupMatchSelectWidth={false}   // ✅ 新写法
                popupClassName="urdf-dropdown"  // ✅ 自定义下拉样式
                style={{ width: "100%" }}
                filterOption={false}               // ⭐ 使用你的 filteredUrdfs
              />
        </div>

        <div style={{ marginTop: 4, minHeight: 20 }}>
          {loadingList && (
            <Space size="small">
              <Spin size="small" />
              <Text type="secondary" style={{ fontSize: 12 }}>
                正在加载 robots/manifest.json ...
              </Text>
            </Space>
          )}
          {listError && (
            <Alert
              style={{ marginTop: 4 }}
              type="error"
              showIcon
              message="URDF 清单加载失败"
              description={listError}
            />
          )}
        </div>
      </div>

      {/* Base / EE，竖向间距减小 */}
      <Space
        direction="vertical"
        style={{ width: "100%", marginBottom: 4 }}
      >
        <div>
          <Text style={{ fontSize: 12 }}>Base link</Text>
          <Input
            value={baseLink}
            onChange={(e) => setBaseLink(e.target.value)}
          />
        </div>

        <div>
          <Text style={{ fontSize: 12 }}>EE link</Text>
          <Input
            value={eeLink}
            onChange={(e) => setEeLink(e.target.value)}
          />
        </div>
      </Space>

      <Divider style={{ margin: "4px 0" }} />
    </div>

    {/* 下半部分：关节 + Handle 操作区，占满剩余高度 */}
    <div
      style={{
        flex: 1,
        minHeight: 0, // 关键：让内部正确滚动
        overflow: "auto",
        paddingRight: 4,
        fontSize: 12,
      }}
    >
      {jointsMeta.map((j, i) => {
        const unitLabel = j.unit === "deg" ? "°" : "m";
        const min = j.lower ?? (j.unit === "deg" ? -180 : -1);
        const max = j.upper ?? (j.unit === "deg" ? 180 : 1);
        const step = j.unit === "deg" ? 0.5 : 0.001;
        const val = clamp(qDeg[i] ?? 0, min, max);
        const finite = Number.isFinite(min) && Number.isFinite(max);

        return (
          <Card
            key={j.name + i}
            size="small"
            style={{ marginBottom: 6 }}
            bodyStyle={{ padding: 6 }}
          >
            {/* 标题行压缩 */}
            <div
              style={{
                display: "flex",
                justifyContent: "space-between",
                marginBottom: 4,
                fontFamily: "ui-monospace, monospace",
                fontSize: 11,
              }}
            >
              <span style={{ fontWeight: 600 }}>{j.name}</span>
              <span style={{ color: "#666" }}>
                [
                {Number.isFinite(min)
                  ? min.toFixed(j.unit === "deg" ? 1 : 3)
                  : "-∞"}
                ,
                {Number.isFinite(max)
                  ? max.toFixed(j.unit === "deg" ? 1 : 3)
                  : "+∞"}
                ]
                {unitLabel} · {j.type}
              </span>
            </div>

            {/* Slider 间距压缩 */}
            <Slider
              min={Number.isFinite(min) ? min : 0}
              max={Number.isFinite(max) ? max : 0}
              step={step}
              value={Number.isFinite(val) ? val : 0}
              onChange={(value) => onJointSlider(i, value as number)}
              disabled={!finite}
              style={{ margin: "0 0 4px" }}
            />

            {/* 输入行压缩 + 小号 InputNumber */}
            <Space style={{ marginTop: 0 }} align="center" size={6}>
              <InputNumber
                size="small"
                value={Number.isFinite(val) ? val : 0}
                step={step}
                min={Number.isFinite(min) ? min : undefined}
                max={Number.isFinite(max) ? max : undefined}
                onChange={(value) => {
                  const num = Number(value ?? 0);
                  onJointSlider(i, clamp(num, min, max));
                }}
                placeholder={j.unit === "deg" ? "角度" : "位移"}
                style={{ width: 110 }}
              />
              <Text type="secondary" style={{ fontSize: 11 }}>
                {unitLabel}
              </Text>
            </Space>
          </Card>
        );
      })}

      {/* Handle 位姿 - 紧凑版 */}
      <Card
        size="small"
        style={{ marginTop: 8, marginBottom: 4 }}
        bodyStyle={{ padding: 6 }}
      >
        <Text strong style={{ fontSize: 12 }}>
          Handle 位姿
        </Text>

        {/* 位置 XYZ：一行三个 */}
        <Space
          style={{ width: "100%", marginTop: 6 }}
          size={8}
          align="center"
        >
          <Space size={4} align="center">
            <Text style={{ width: 24 }}>X</Text>
            <InputNumber
              size="small"
              step={0.001}
              value={handlePos[0]}
              onChange={(value) => {
                const x = Number(value ?? 0);
                const next: [number, number, number] = [
                  x,
                  handlePos[1],
                  handlePos[2],
                ];
                setHandlePos(next);
                viewerRef.current?.setHandlePose({ position: next });
              }}
              style={{ width: 90 }}
            />
          </Space>

          <Space size={4} align="center">
            <Text style={{ width: 24 }}>Y</Text>
            <InputNumber
              size="small"
              step={0.001}
              value={handlePos[1]}
              onChange={(value) => {
                const y = Number(value ?? 0);
                const next: [number, number, number] = [
                  handlePos[0],
                  y,
                  handlePos[2],
                ];
                setHandlePos(next);
                viewerRef.current?.setHandlePose({ position: next });
              }}
              style={{ width: 90 }}
            />
          </Space>

          <Space size={4} align="center">
            <Text style={{ width: 24 }}>Z</Text>
            <InputNumber
              size="small"
              step={0.001}
              value={handlePos[2]}
              onChange={(value) => {
                const z = Number(value ?? 0);
                const next: [number, number, number] = [
                  handlePos[0],
                  handlePos[1],
                  z,
                ];
                setHandlePos(next);
                viewerRef.current?.setHandlePose({ position: next });
              }}
              style={{ width: 90 }}
            />
          </Space>
        </Space>

        {/* 姿态 RPY：同样一行三个 */}
        <Space
          style={{ width: "100%", marginTop: 6 }}
          size={8}
          align="center"
        >
          <Space size={4} align="center">
            <Text style={{ width: 48 }}>Roll</Text>
            <InputNumber
              size="small"
              step={0.5}
              value={handleEulerDeg[0]}
              onChange={(value) => {
                const r = Number(value ?? 0);
                const next: [number, number, number] = [
                  r,
                  handleEulerDeg[1],
                  handleEulerDeg[2],
                ];
                setHandleEulerDeg(next);
                viewerRef.current?.setHandlePose({ eulerDeg: next });
              }}
              style={{ width: 90 }}
            />
          </Space>

          <Space size={4} align="center">
            <Text style={{ width: 48 }}>Pitch</Text>
            <InputNumber
              size="small"
              step={0.5}
              value={handleEulerDeg[1]}
              onChange={(value) => {
                const p = Number(value ?? 0);
                const next: [number, number, number] = [
                  handleEulerDeg[0],
                  p,
                  handleEulerDeg[2],
                ];
                setHandleEulerDeg(next);
                viewerRef.current?.setHandlePose({ eulerDeg: next });
              }}
              style={{ width: 90 }}
            />
          </Space>

          <Space size={4} align="center">
            <Text style={{ width: 48 }}>Yaw</Text>
            <InputNumber
              size="small"
              step={0.5}
              value={handleEulerDeg[2]}
              onChange={(value) => {
                const y = Number(value ?? 0);
                const next: [number, number, number] = [
                  handleEulerDeg[0],
                  handleEulerDeg[1],
                  y,
                ];
                setHandleEulerDeg(next);
                viewerRef.current?.setHandlePose({ eulerDeg: next });
              }}
              style={{ width: 90 }}
            />
          </Space>
        </Space>

        {/* 操作按钮也压缩一下间距 & 尺寸 */}
        <Space style={{ marginTop: 6 }} size={8}>
          <Button
            type="primary"
            size="small"
            onClick={() => {
              const target = {
                position: [
                  handlePos[0],
                  handlePos[1],
                  handlePos[2] - 0.05,
                ] as [number, number, number],
                euler: handleEulerDeg.map(
                  (d) => (d * Math.PI) / 180
                ) as [number, number, number],
              };
              viewerRef.current?.startMoveLinear(target, {
                linear: 0.25,
                angularDeg: 60,
                smooth: true,
                rateScale: 1.0,
                tauPos: 0.02,
                tauRot: 0.02,
                preferAnalytic: true,
              });
            }}
          >
            直线 +5cm
          </Button>

          <Button
            size="small"
            onClick={() => {
              viewerRef.current?.startMoveJoint(
                [0, 0, 0, 0, 0, 0],
                { speed: 1, smooth: true }
              );
            }}
          >
            movejoint
          </Button>
        </Space>
      </Card>
    </div>
  </div>

  {/* 右侧 Viewer 区域，保持不变，占满剩余宽度 & 高度 */}
  <div
    style={{
      position: "relative",
      flex: 1,
      background: "#0b0b0b",
    }}
  >
    <URDFViewer
      ref={viewerRef}
      urdfUrl={urdfUrl}
      baseLink={baseLink}
      eeLink={eeLink}
      qDeg={qDeg}
      onJointsReady={onJointsReady}
      onQChange={onQChangeFromViewer}
      onHandleChange={(pose) => {
        setHandlePos(pose.position);
        setHandleEulerDeg(pose.eulerDeg);
      }}
    />
  </div>
</div>

  );
}
