import { useState, useRef, useLayoutEffect, useEffect } from 'preact/hooks';
import styles from '../../icon.module-K7UO6GPK.module.css';
import { jsxs, jsx, Fragment } from 'preact/jsx-runtime';
import styles2 from '../../collapsible-panel.module-ZOYR4Y52.module.css';

// src/widgets/collapsible-panel/collapsible-panel.tsx

// src/utils/cx/index.ts
function cx(...args) {
  let str = "";
  let i = 0;
  for (i = 0; i < args.length; i++) {
    const arg = args[i];
    if (arg) {
      if (typeof arg === "string") {
        str += (str ? " " : "") + arg;
      } else if (typeof arg === "object") {
        for (const key in arg) {
          if (arg[key]) {
            str += (str ? " " : "") + key;
          }
        }
      }
    }
  }
  return str;
}
var Icon = (props) => {
  const typeClass = props.type ? styles[`${props.type}-icon`] : null;
  const classes = cx(styles.ic, styles[`s-${props.size || "m"}`], typeClass, props.classes);
  const wrapperClasses = cx(styles.wrapper, {
    [styles.iconDisabled]: props.isDisabled
  });
  return /* @__PURE__ */ jsxs("div", { className: wrapperClasses, onClick: props.onClick, style: props.style, children: [
    /* @__PURE__ */ jsx("svg", { className: classes, children: /* @__PURE__ */ jsx("use", { xlinkHref: `#${props.name}` }) }),
    props.subscript && /* @__PURE__ */ jsx("span", { className: cx(styles.badge), children: props.subscript })
  ] });
};
var CollapsiblePanel = ({
  header,
  headerComponent,
  icon,
  children,
  defaultCollapsed = false,
  className
}) => {
  const [collapsed, setCollapsed] = useState(defaultCollapsed);
  const [height, setHeight] = useState(collapsed ? 0 : "auto");
  const [shouldRender, setShouldRender] = useState(!defaultCollapsed);
  const cntRef = useRef(null);
  const btnRef = useRef(null);
  const rippleTimeoutRef = useRef(null);
  useLayoutEffect(() => {
    if (collapsed) {
      setHeight(cntRef.current ? cntRef.current.scrollHeight : 0);
      requestAnimationFrame(() => setHeight(0));
    } else {
      setShouldRender(true);
      setHeight(cntRef.current ? cntRef.current.scrollHeight : "auto");
      const timeout = setTimeout(() => setHeight("auto"), 200);
      return () => clearTimeout(timeout);
    }
  }, [collapsed]);
  useEffect(() => {
    if (collapsed) {
      const timeout = setTimeout(() => setShouldRender(false), 200);
      return () => clearTimeout(timeout);
    }
  }, [collapsed]);
  const handleMouseDown = (e) => {
    const button = btnRef.current;
    if (!button) return;
    const ripple = document.createElement("span");
    const rect = button.getBoundingClientRect();
    const size = Math.max(rect.width, rect.height);
    const x = e.clientX - rect.left - size / 2;
    const y = e.clientY - rect.top - size / 2;
    ripple.style.width = ripple.style.height = `${size}px`;
    ripple.style.left = `${x}px`;
    ripple.style.top = `${y}px`;
    ripple.className = "ripple-base";
    button.appendChild(ripple);
    if (rippleTimeoutRef.current !== null) {
      clearTimeout(rippleTimeoutRef.current);
    }
    rippleTimeoutRef.current = window.setTimeout(() => {
      ripple.remove();
      rippleTimeoutRef.current = null;
    }, 600);
  };
  useEffect(() => {
    return () => {
      if (rippleTimeoutRef.current !== null) {
        clearTimeout(rippleTimeoutRef.current);
        rippleTimeoutRef.current = null;
      }
    };
  }, []);
  return /* @__PURE__ */ jsxs("div", { className: cx("w-full bg-overlay card-base overflow-hidden", className), children: [
    /* @__PURE__ */ jsxs(
      "button",
      {
        ref: btnRef,
        className: cx(styles2.btn, collapsed ? styles2.closed : styles2.open),
        onClick: () => setCollapsed((c) => !c),
        onMouseDown: handleMouseDown,
        type: "button",
        "aria-expanded": !collapsed,
        children: [
          /* @__PURE__ */ jsx("span", { className: styles2.header, children: headerComponent ? headerComponent : /* @__PURE__ */ jsxs(Fragment, { children: [
            icon && /* @__PURE__ */ jsx(Icon, { name: icon, size: "s" }),
            header
          ] }) }),
          /* @__PURE__ */ jsx("span", { className: cx(styles2.icon, !collapsed ? styles2.iconOpen : styles2.iconClosed), children: /* @__PURE__ */ jsx(Icon, { name: "arrow_right", size: "m" }) })
        ]
      }
    ),
    /* @__PURE__ */ jsx(
      "div",
      {
        style: {
          overflow: "hidden",
          height: typeof height === "number" ? `${height}px` : height,
          opacity: collapsed ? 0 : 1,
          transition: "height 0.3s ease-in-out, opacity 0.2s ease-in-out"
        },
        children: shouldRender && /* @__PURE__ */ jsx("div", { ref: cntRef, className: "p-3", children })
      }
    )
  ] });
};
var collapsible_panel_default = CollapsiblePanel;

export { CollapsiblePanel, collapsible_panel_default as default };
//# sourceMappingURL=collapsible-panel.js.map
//# sourceMappingURL=collapsible-panel.js.map