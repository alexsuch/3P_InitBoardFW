#!/usr/bin/env node
import { promises as fs } from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { optimize } from 'svgo';
import { pathToFileURL } from 'node:url';

const resolve = (...segments) => path.resolve(process.cwd(), ...segments);

const parseArgs = () => {
  const args = process.argv.slice(2);
  const options = {
    out: resolve('packages/micro-ui/dist/icon-sprite.svg'),
    sources: [],
    optimize: true,
    svgoConfigPath: undefined,
    useTemplate: false,
  };

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];
    if (arg === '--out' && args[i + 1]) {
      options.out = resolve(args[++i]);
    } else if (arg === '--src' && args[i + 1]) {
      options.sources.push(resolve(args[++i]));
    } else if (arg === '--no-opt' || arg === '--no-optimize' || arg === '--svgo-off') {
      options.optimize = false;
    } else if ((arg === '--svgo-config' || arg === '--svgo') && args[i + 1]) {
      options.svgoConfigPath = resolve(args[++i]);
    } else if (arg === '--use-template' || arg === '--svgo-template') {
      options.useTemplate = true;
    }
  }

  return options;
};

const sanitizeSymbolId = (name) => {
  const normalized = name
    .toLowerCase()
    .replace(/[^a-z0-9\-_.:]/g, '-')
    .replace(/-+/g, '-')
    .replace(/^-+/, '')
    .replace(/-+$/, '');

  if (normalized) {
    return normalized;
  }

  const hash = createHash('sha1').update(name).digest('hex').slice(0, 8);
  return `icon-${hash}`;
};

// Prefix all IDs and ID references in SVG content to avoid collisions
const prefixIds = (svgContent, prefix) => {
  // Create a map of old IDs to new prefixed IDs
  const idMap = new Map();

  // Find all id attributes and prefix them
  svgContent = svgContent.replace(/\bid="([^"]+)"/gi, (match, id) => {
    const prefixedId = `${prefix}-${id}`;
    idMap.set(id, prefixedId);
    return `id="${prefixedId}"`;
  });

  // Update all references to IDs (xlink:href, href, url(), etc.)
  idMap.forEach((newId, oldId) => {
    // Update xlink:href="#id" and href="#id"
    svgContent = svgContent.replace(
      new RegExp(`(xlink:)?href="#${oldId.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}"`, 'gi'),
      `$1href="#${newId}"`,
    );
    // Update url(#id) references
    svgContent = svgContent.replace(new RegExp(`url\\(#${oldId.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\)`, 'gi'), `url(#${newId})`);
  });

  return svgContent;
};

const extractSymbolMarkup = (svgContent, symbolId, svgoOptions) => {
  let safeSvg = svgContent;
  const origViewBoxMatch = svgContent.match(/viewBox="([^"]*)"/i);
  const origViewBox = origViewBoxMatch ? origViewBoxMatch[1] : null;

  // When svgoOptions is null, skip optimization entirely
  if (svgoOptions !== null) {
    const { data } = optimize(svgContent, svgoOptions);
    safeSvg = data;
  }

  const match = safeSvg.match(/<svg([^>]*)>([\s\S]*?)<\/svg>/i);
  if (!match) {
    throw new Error(`Icon ${symbolId} is not a valid SVG file.`);
  }

  const [, rawAttributes, rawBody] = match;
  // Strip attributes that should not be carried over to <symbol>
  const cleanedAttributes = rawAttributes
    .replace(/\s+xmlns=\"[^\"]*\"/gi, '')
    .replace(/\s+on[a-z]+\s*=\s*"[^"]*"/gi, '')
    .replace(/\s+viewBox=\"[^\"]*\"/gi, '')
    .replace(/\s+width=\"[^\"]*\"/gi, '')
    .replace(/\s+height=\"[^\"]*\"/gi, '')
    .replace(/\s+preserveAspectRatio=\"[^\"]*\"/gi, '')
    .trim();
  const viewBoxMatch = rawAttributes.match(/viewBox=\"([^\"]*)\"/i);
  const viewBox = viewBoxMatch ? viewBoxMatch[1] : origViewBox || '0 0 24 24';

  const attrString = [`viewBox=\"${viewBox}\"`, `preserveAspectRatio=\"xMidYMid meet\"`, cleanedAttributes]
    .filter(Boolean)
    .join(' ')
    .trim();

  // Prefix all IDs in the body to avoid collisions between symbols
  const body = prefixIds(rawBody.trim(), symbolId);

  return `<symbol id="${symbolId}" ${attrString}>${body}</symbol>`;
};

const readSvg = async (iconPath, symbolId, svgoOptions) => {
  const content = await fs.readFile(iconPath, 'utf8');
  return extractSymbolMarkup(content, symbolId, svgoOptions);
};

const collectIcons = async (options) => {
  const icons = new Map();
  const svgoOptions = options.svgo;

  // Load icons only from provided source directories (later sources override earlier ones)
  for (const srcDir of options.sources) {
    const exists = await fs
      .stat(srcDir)
      .then(() => true)
      .catch(() => false);
    if (!exists) continue;

    const files = await fs.readdir(srcDir);
    for (const file of files) {
      if (!file.endsWith('.svg') || file === 'icon-sprite.svg') continue; // Skip non-SVG and generated sprite
      const rawName = path.basename(file, '.svg');
      const sanitizedName = sanitizeSymbolId(rawName);
      const filePath = path.join(srcDir, file);
      const symbol = await readSvg(filePath, sanitizedName, svgoOptions);
      icons.set(sanitizedName, symbol);
    }
  }

  return icons;
};

// Built-in SVGO configuration template. Start with safe defaults and
// uncomment plugins one-by-one to debug issues with specific icons.
// Tip: keep removeViewBox disabled (false) so icons scale properly.
// Note: Param shapes below are typical; consult SVGO docs for full details.
const svgoTemplate = () => ({
  multipass: true,
  // js2svg: { indent: 2, pretty: true },
  floatPrecision: 3,
  // plugins: [
  //   // Use preset-default and selectively disable problematic transforms here
  //   {
  //     name: 'preset-default',
  //     params: {
  //       overrides: {
  //         removeViewBox: false,
  //         // cleanupIDs: false,
  //         // convertShapeToPath: false,
  //         // mergePaths: false,
  //         // convertPathData: { floatPrecision: 3 },
  //         // convertTransform: { floatPrecision: 3 },
  //       },
  //     },
  //   },
  //   // Base cleanup/removal plugins (usually safe)
  //   { name: 'removeDoctype' },
  //   { name: 'removeXMLProcInst' },
  //   { name: 'removeComments' },
  //   { name: 'removeMetadata' },
  //   { name: 'removeEditorsNSData' },
  //   { name: 'cleanupAttrs' },
  //   { name: 'mergeStyles' },
  //   { name: 'inlineStyles', params: { onlyMatchedOnce: false, removeMatchedSelectors: true } },
  //   { name: 'minifyStyles' },
  //   // ID / namespace handling
  //   { name: 'cleanupIDs', params: { remove: true, minify: true } },
  //   { name: 'removeUnusedNS' },
  //   // Definitions and unknowns
  //   { name: 'removeUselessDefs' },
  //   { name: 'removeUnknownsAndDefaults', params: { unknownContent: true, unknownAttrs: true, defaultAttrs: true } },
  //   // Group/attrs cleanups
  //   { name: 'removeNonInheritableGroupAttrs' },
  //   { name: 'removeUselessStrokeAndFill' },
  //   // ViewBox and dimensions
  //   { name: 'removeViewBox' }, // keep disabled via preset override
  //   { name: 'removeDimensions' },
  //   { name: 'cleanupEnableBackground' },
  //   // Hidden/empty removals
  //   { name: 'removeHiddenElems' },
  //   { name: 'removeEmptyText' },
  //   { name: 'removeEmptyAttrs' },
  //   { name: 'removeEmptyContainers' },
  //   // Shape/transform/path conversions
  //   { name: 'convertShapeToPath', params: { convertArcs: true } },
  //   { name: 'convertEllipseToCircle' },
  //   { name: 'convertPathData', params: { floatPrecision: 3, transformPrecision: 3, noSpaceAfterFlags: false } },
  //   { name: 'convertTransform', params: { floatPrecision: 3 } },
  //   // Grouping
  //   { name: 'moveElemsAttrsToGroup' },
  //   { name: 'moveGroupAttrsToElems' },
  //   { name: 'collapseGroups' },
  //   // Colors
  //   { name: 'convertColors', params: { currentColor: false, shorthex: true, shortname: true } },
  //   // Sorting
  //   { name: 'sortAttrs', params: { xmlnsOrder: 'alphabetical' } },
  //   { name: 'sortDefsChildren' },
  //   // Paths merging
  //   { name: 'mergePaths', params: { force: false } },
  //   { name: 'reusePaths' },
  //   // Attrs and elements manipulation
  //   { name: 'removeAttrs', params: { attrs: ['on.*', 'data-.*'] } },
  //   { name: 'removeElementsByAttr', params: { id: 'elementId' } },
  //   { name: 'addAttributesToSVGElement', params: { attributes: [{ focusable: 'false' }] } },
  //   { name: 'addClassesToSVGElement', params: { className: ['my-svg'] } },
  //   // Title/desc
  //   { name: 'removeTitle' },
  //   { name: 'removeDesc' },
  //   // Raster and scripts/styles (be careful)
  //   { name: 'removeRasterImages' },
  //   { name: 'removeStyleElement' },
  //   { name: 'removeScriptElement' },
  //   // Prefix IDs and class names to avoid collisions
  //   { name: 'prefixIds', params: { prefix: '', delim: '-', prefixIds: true, prefixClassNames: true } },
  // ],
});

const loadSvgoConfig = async (configPath) => {
  if (!configPath) return undefined;
  const lc = configPath.toLowerCase();
  if (lc.endsWith('.json')) {
    const raw = await fs.readFile(configPath, 'utf8');
    return JSON.parse(raw);
  }
  if (lc.endsWith('.mjs') || lc.endsWith('.js') || lc.endsWith('.cjs')) {
    const mod = await import(pathToFileURL(configPath).href);
    return mod.default || mod;
  }
  // Fallback try JSON
  const raw = await fs.readFile(configPath, 'utf8');
  return JSON.parse(raw);
};

const buildSprite = async () => {
  const options = parseArgs();
  // Prepare SVGO options
  let svgoOptions = null; // null => skip optimize; object => pass to svgo
  if (options.optimize === false) {
    svgoOptions = null;
  } else if (options.useTemplate) {
    svgoOptions = svgoTemplate();
  } else if (options.svgoConfigPath) {
    try {
      const loaded = await loadSvgoConfig(options.svgoConfigPath);
      // Ensure multipass true unless explicitly overridden
      if (loaded && typeof loaded === 'object' && !Object.prototype.hasOwnProperty.call(loaded, 'multipass')) {
        loaded.multipass = true;
      }
      svgoOptions = loaded || { multipass: true };
    } catch (e) {
      console.warn(`micro-ui: failed to read svgo config at ${options.svgoConfigPath}, falling back to defaults`);
      svgoOptions = { multipass: true };
    }
  } else {
    // Default behavior: SVGO with default preset
    svgoOptions = { multipass: true };
  }

  // Expose computed svgo options to downstream functions
  options.svgo = svgoOptions;
  const icons = await collectIcons(options);
  const spriteContent = `<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" aria-hidden="true" style="position:absolute;width:0;height:0;overflow:hidden">\n${Array.from(
    icons.values(),
  ).join('\n')}\n</svg>\n`;

  await fs.mkdir(path.dirname(options.out), { recursive: true });
  await fs.writeFile(options.out, spriteContent, 'utf8');

  console.log(`micro-ui: generated sprite with ${icons.size} icons -> ${options.out}`);
};

buildSprite().catch((error) => {
  console.error('micro-ui: failed to build icon sprite');
  console.error(error);
  process.exit(1);
});
