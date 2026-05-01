import fs from "fs";
import path from "path";

function stripQuotes(value) {
  if (
    (value.startsWith('"') && value.endsWith('"')) ||
    (value.startsWith("'") && value.endsWith("'"))
  ) {
    return value.slice(1, -1);
  }
  return value;
}

function parseScalar(rawValue) {
  const value = String(rawValue ?? "").trim();
  if (!value) return "";

  if (value === "true") return true;
  if (value === "false") return false;
  if (value === "null") return null;

  if (
    (value.startsWith("[") && value.endsWith("]")) ||
    (value.startsWith("{") && value.endsWith("}"))
  ) {
    try {
      return JSON.parse(value.replace(/'/g, '"'));
    } catch (_err) {
      return value;
    }
  }

  const numberValue = Number(value);
  if (!Number.isNaN(numberValue) && value !== "") {
    return numberValue;
  }

  return stripQuotes(value);
}

function preprocessYaml(text) {
  return String(text)
    .split(/\r?\n/)
    .map((raw) => {
      const line = raw.replace(/\t/g, "    ");
      const trimmed = line.trim();
      if (!trimmed || trimmed.startsWith("#")) return null;
      return {
        indent: line.match(/^ */)[0].length,
        content: trimmed,
      };
    })
    .filter(Boolean);
}

function inferContainer(lines, currentIndex, currentIndent) {
  for (let i = currentIndex + 1; i < lines.length; i += 1) {
    const line = lines[i];
    if (line.indent <= currentIndent) break;
    return line.content.startsWith("- ") ? [] : {};
  }
  return {};
}

function processMapping(parent, content, lines, currentIndex, indent, stack) {
  const separator = content.indexOf(":");
  if (separator < 0) {
    return;
  }

  const key = content.slice(0, separator).trim();
  const rawValue = content.slice(separator + 1).trim();
  if (!key) return;

  if (!rawValue) {
    const child = inferContainer(lines, currentIndex, indent);
    parent[key] = child;
    if (child && typeof child === "object") {
      stack.push({ indent, container: child });
    }
    return;
  }

  parent[key] = parseScalar(rawValue);
}

function parseYamlSubset(text) {
  const lines = preprocessYaml(text);
  const root = {};
  const stack = [{ indent: -1, container: root }];

  for (let index = 0; index < lines.length; index += 1) {
    const { indent, content } = lines[index];

    while (stack.length > 1 && indent <= stack[stack.length - 1].indent) {
      stack.pop();
    }

    const parent = stack[stack.length - 1].container;

    if (content.startsWith("- ")) {
      if (!Array.isArray(parent)) {
        throw new Error(`Invalid YAML structure near "${content}"`);
      }

      const itemContent = content.slice(2).trim();
      if (!itemContent) {
        const child = inferContainer(lines, index, indent);
        parent.push(child);
        if (child && typeof child === "object") {
          stack.push({ indent, container: child });
        }
        continue;
      }

      if (itemContent.includes(":")) {
        const obj = {};
        parent.push(obj);
        stack.push({ indent, container: obj });
        processMapping(obj, itemContent, lines, index, indent, stack);
        continue;
      }

      parent.push(parseScalar(itemContent));
      continue;
    }

    processMapping(parent, content, lines, index, indent, stack);
  }

  return root;
}

function asNumber(value, fallback = 0.0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function asArray(value) {
  return Array.isArray(value) ? value : [];
}

function asString(value, fallback = "") {
  return value == null ? fallback : String(value);
}

function normalizeSemanticId(value, mapName) {
  const raw = asString(value).trim();
  if (!raw) return mapName;
  const legacyPrefix = `${mapName}_v`;
  if (raw.startsWith(legacyPrefix)) return mapName;
  return raw;
}

function normalizePose(value, fallback = {}) {
  return {
    x: asNumber(value?.x, fallback.x ?? 0.0),
    y: asNumber(value?.y, fallback.y ?? 0.0),
    z: asNumber(value?.z, fallback.z ?? 0.0),
    yaw: asNumber(value?.yaw, fallback.yaw ?? 0.0),
  };
}

function parsePgmHeader(buffer) {
  const bytes = new Uint8Array(buffer);
  let offset = 0;

  function skipSpaceAndComments() {
    while (offset < bytes.length) {
      const ch = String.fromCharCode(bytes[offset]);
      if (/\s/.test(ch)) {
        offset += 1;
        continue;
      }
      if (ch === "#") {
        while (offset < bytes.length && bytes[offset] !== 10 && bytes[offset] !== 13) {
          offset += 1;
        }
        continue;
      }
      break;
    }
  }

  function nextToken() {
    skipSpaceAndComments();
    const start = offset;
    while (offset < bytes.length) {
      const ch = String.fromCharCode(bytes[offset]);
      if (/\s/.test(ch) || ch === "#") break;
      offset += 1;
    }
    return Buffer.from(bytes.subarray(start, offset)).toString("ascii");
  }

  const magic = nextToken();
  const width = Number(nextToken());
  const height = Number(nextToken());
  const maxValue = Number(nextToken());
  skipSpaceAndComments();

  if (magic !== "P5" || !Number.isFinite(width) || !Number.isFinite(height)) {
    throw new Error("Unsupported or invalid PGM file");
  }

  return { width, height, maxValue, dataOffset: offset };
}

function getSemanticFilePath(repoRoot) {
  return path.join(
    repoRoot,
    "workspace",
    "src",
    "robot_navigation",
    "config",
    "semantic_map_ECEMain.yaml"
  );
}

function normalizeAnchor(anchor) {
  return {
    id: asString(anchor?.id),
    label: asString(anchor?.label, asString(anchor?.id)),
    type: asString(anchor?.type, "anchor"),
    x: asNumber(anchor?.x),
    y: asNumber(anchor?.y),
    yaw: asNumber(anchor?.yaw),
  };
}

function normalizeRack(rack) {
  return {
    id: asString(rack?.id),
    label: asString(rack?.label, asString(rack?.id)),
    anchor_id: asString(rack?.anchor_id),
    x: asNumber(rack?.x),
    y: asNumber(rack?.y),
    yaw: asNumber(rack?.yaw),
    width: asNumber(rack?.width, 0.8),
    depth: asNumber(rack?.depth, 0.4),
    levels: asNumber(rack?.levels, 1),
  };
}

function normalizeSlot(slot, anchorsById) {
  const anchor = anchorsById.get(asString(slot?.anchor_id)) || null;
  const fallbackPose = anchor
    ? { x: anchor.x, y: anchor.y, z: 0.0, yaw: anchor.yaw }
    : { x: 0.0, y: 0.0, z: 0.0, yaw: 0.0 };

  return {
    id: asString(slot?.id),
    label: asString(slot?.label, asString(slot?.id)),
    rack_id: asString(slot?.rack_id),
    anchor_id: asString(slot?.anchor_id),
    rack_level: asNumber(slot?.rack_level, 0),
    product_ids: asArray(slot?.product_ids).map((value) => asString(value)).filter(Boolean),
    product_names: asArray(slot?.product_names).map((value) => asString(value)).filter(Boolean),
    nav_pose: normalizePose(slot?.nav_pose, fallbackPose),
    service_pose: normalizePose(slot?.service_pose, fallbackPose),
  };
}

function normalizeBundleForSave(bundle) {
  const mapName = asString(bundle?.map?.name, "ECEMain");
  const anchors = asArray(bundle?.anchors).map(normalizeAnchor).filter((item) => item.id);
  const anchorsById = new Map(anchors.map((item) => [item.id, item]));
  const racks = asArray(bundle?.racks).map(normalizeRack).filter((item) => item.id);
  const slots = asArray(bundle?.slots)
    .map((slot) => normalizeSlot(slot, anchorsById))
    .filter((item) => item.id);

  return {
    map: {
      name: mapName,
      frame_id: asString(bundle?.map?.frame_id, "map"),
      semantic_id: normalizeSemanticId(bundle?.map?.semantic_id, mapName),
    },
    anchors,
    racks,
    slots,
  };
}

function formatNumber(value) {
  const normalized = asNumber(value);
  const text = normalized.toFixed(6).replace(/\.?0+$/, "");
  return text.includes(".") ? text : `${text}.0`;
}

function formatInlineStringArray(values) {
  const items = asArray(values)
    .map((value) => asString(value).trim())
    .filter(Boolean)
    .map((value) => JSON.stringify(value));
  return `[${items.join(", ")}]`;
}

function pushPoseYaml(lines, indent, key, pose, includeZ) {
  lines.push(`${indent}${key}:`);
  lines.push(`${indent}  x: ${formatNumber(pose?.x)}`);
  lines.push(`${indent}  y: ${formatNumber(pose?.y)}`);
  if (includeZ) {
    lines.push(`${indent}  z: ${formatNumber(pose?.z)}`);
  }
  lines.push(`${indent}  yaw: ${formatNumber(pose?.yaw)}`);
}

function serializeSemanticMap(bundle) {
  const normalized = normalizeBundleForSave(bundle);
  const lines = [
    "map:",
    `  name: ${normalized.map.name}`,
    `  frame_id: ${normalized.map.frame_id}`,
    `  semantic_id: ${normalized.map.semantic_id}`,
    "",
    "anchors:",
  ];

  for (const anchor of normalized.anchors) {
    lines.push(`  - id: ${anchor.id}`);
    lines.push(`    label: ${anchor.label}`);
    lines.push(`    type: ${anchor.type}`);
    lines.push(`    x: ${formatNumber(anchor.x)}`);
    lines.push(`    y: ${formatNumber(anchor.y)}`);
    lines.push(`    yaw: ${formatNumber(anchor.yaw)}`);
  }

  lines.push("");
  lines.push("racks:");
  for (const rack of normalized.racks) {
    lines.push(`  - id: ${rack.id}`);
    lines.push(`    label: ${rack.label}`);
    lines.push(`    anchor_id: ${rack.anchor_id}`);
    lines.push(`    x: ${formatNumber(rack.x)}`);
    lines.push(`    y: ${formatNumber(rack.y)}`);
    lines.push(`    yaw: ${formatNumber(rack.yaw)}`);
    lines.push(`    width: ${formatNumber(rack.width)}`);
    lines.push(`    depth: ${formatNumber(rack.depth)}`);
    lines.push(`    levels: ${Math.max(1, Math.round(asNumber(rack.levels, 1)))}`);
  }

  lines.push("");
  lines.push("slots:");
  for (const slot of normalized.slots) {
    lines.push(`  - id: ${slot.id}`);
    lines.push(`    label: ${slot.label}`);
    lines.push(`    rack_id: ${slot.rack_id}`);
    lines.push(`    anchor_id: ${slot.anchor_id}`);
    lines.push(`    rack_level: ${Math.max(0, Math.round(asNumber(slot.rack_level, 0)))}`);
    lines.push(`    product_ids: ${formatInlineStringArray(slot.product_ids)}`);
    lines.push(`    product_names: ${formatInlineStringArray(slot.product_names)}`);
    pushPoseYaml(lines, "    ", "nav_pose", slot.nav_pose, false);
    pushPoseYaml(lines, "    ", "service_pose", slot.service_pose, true);
  }

  lines.push("");
  return lines.join("\n");
}

let cachedBundle = null;
let cachedKey = "";

export function getMapPgmPath(repoRoot, mapName) {
  const safeMapName = String(mapName || "").trim();
  if (!/^[A-Za-z0-9_-]+$/.test(safeMapName)) {
    throw new Error("Invalid map name");
  }
  return path.join(repoRoot, "Maps", `${safeMapName}.pgm`);
}

export function getSemanticMapBundle(repoRoot) {
  const semanticFilePath = getSemanticFilePath(repoRoot);
  const semanticRaw = parseYamlSubset(fs.readFileSync(semanticFilePath, "utf8"));
  const mapName = asString(semanticRaw?.map?.name, "ECEMain");
  const mapYamlPath = path.join(repoRoot, "Maps", `${mapName}.yaml`);
  const mapPgmPath = getMapPgmPath(repoRoot, mapName);
  const cacheKey = [semanticFilePath, mapYamlPath, mapPgmPath]
    .map((filePath) => `${filePath}:${fs.statSync(filePath).mtimeMs}`)
    .join("|");

  if (cachedBundle && cachedKey === cacheKey) {
    return cachedBundle;
  }

  const mapMeta = parseYamlSubset(fs.readFileSync(mapYamlPath, "utf8"));
  const pgmHeader = parsePgmHeader(fs.readFileSync(mapPgmPath));
  const origin = asArray(mapMeta?.origin).map((value) => asNumber(value));
  while (origin.length < 3) origin.push(0.0);

  const anchors = asArray(semanticRaw?.anchors).map(normalizeAnchor).filter((item) => item.id);
  const anchorsById = new Map(anchors.map((item) => [item.id, item]));
  const racks = asArray(semanticRaw?.racks).map(normalizeRack).filter((item) => item.id);
  const slots = asArray(semanticRaw?.slots)
    .map((slot) => normalizeSlot(slot, anchorsById))
    .filter((item) => item.id);

  const resolution = asNumber(mapMeta?.resolution, 0.03);
  const bundle = {
    map: {
      name: mapName,
      frame_id: asString(semanticRaw?.map?.frame_id, "map"),
      semantic_id: normalizeSemanticId(semanticRaw?.map?.semantic_id, mapName),
      resolution,
      origin,
      negate: asNumber(mapMeta?.negate, 0),
      occupied_thresh: asNumber(mapMeta?.occupied_thresh, 0.65),
      free_thresh: asNumber(mapMeta?.free_thresh, 0.196),
      width_px: pgmHeader.width,
      height_px: pgmHeader.height,
      image_url: `/api/maps/base/${mapName}.pgm`,
      image_format: "pgm",
      bounds: {
        min_x: origin[0],
        min_y: origin[1],
        max_x: origin[0] + pgmHeader.width * resolution,
        max_y: origin[1] + pgmHeader.height * resolution,
      },
    },
    anchors,
    racks,
    slots,
    summary: {
      anchor_count: anchors.length,
      rack_count: racks.length,
      slot_count: slots.length,
    },
    source_files: {
      semantic_map: semanticFilePath,
      base_map_yaml: mapYamlPath,
      base_map_pgm: mapPgmPath,
    },
  };

  cachedBundle = bundle;
  cachedKey = cacheKey;
  return bundle;
}

export function saveSemanticMapBundle(repoRoot, nextBundle) {
  const semanticFilePath = getSemanticFilePath(repoRoot);
  const yamlText = serializeSemanticMap(nextBundle);
  fs.writeFileSync(semanticFilePath, yamlText, "utf8");
  cachedBundle = null;
  cachedKey = "";
  return getSemanticMapBundle(repoRoot);
}
