import { useRef, useEffect } from 'preact/hooks';
import { useSignal } from '@preact/signals';
import styles2 from '../../button.module-APT35VFH.module.css';
import styles from '../../icon.module-K7UO6GPK.module.css';
import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/sliding-panel/sliding-panel.tsx

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
var Button = ({
  variant = "elevated",
  color = "primary",
  iconOnly = false,
  iconAlignment = "left",
  size = "m",
  iconName,
  iconSize,
  iconClassName,
  className,
  children,
  text,
  onClick,
  onMouseDown,
  onMouseUp,
  onPointerDown,
  id,
  disabled,
  isDisabled,
  ...rest
}) => {
  const btnRef = useRef(null);
  const isDisabledState = disabled ?? isDisabled;
  const createRipple = (e) => {
    const button = btnRef.current;
    if (!button || isDisabledState) return;
    const ripple = document.createElement("span");
    const rect = button.getBoundingClientRect();
    const size2 = Math.max(rect.width, rect.height);
    const x = e.clientX - rect.left - size2 / 2;
    const y = e.clientY - rect.top - size2 / 2;
    ripple.style.width = ripple.style.height = `${size2}px`;
    ripple.style.left = `${x}px`;
    ripple.style.top = `${y}px`;
    ripple.className = "ripple-base";
    button.appendChild(ripple);
    setTimeout(() => {
      ripple.remove();
    }, 600);
  };
  const handleMouseDown = (e) => {
    createRipple(e);
    if (onMouseDown) {
      onMouseDown(e);
    }
  };
  const handlePointerDown = (e) => {
    createRipple(e);
    if (onPointerDown) {
      onPointerDown(e);
    }
  };
  const isIconOnly = iconOnly || iconName && !text && !children;
  const showText = !isIconOnly && (text || children);
  const alignmentClass = isIconOnly || !iconName ? "justify-center text-center" : iconAlignment === "left" ? "justify-start text-left" : iconAlignment === "right" ? "justify-end text-right" : "justify-center text-center";
  const colorMap = {
    primary: "cp",
    secondary: "cs",
    warning: "cw",
    success: "csu",
    error: "ce",
    inherit: "ci"
  };
  const variantMap = {
    filled: "vf",
    elevated: "ve",
    outlined: "vo",
    text: "vt",
    ghost: "vg"
  };
  const sizeMap = {
    xs: "s-xs",
    s: "s-s",
    m: "s-m",
    l: "s-l",
    xl: "s-xl"
  };
  const classnames = cx(
    className,
    styles2.b,
    styles2[colorMap[color]],
    styles2[variantMap[variant]],
    styles2[sizeMap[size]],
    { [styles2.io]: isIconOnly },
    "flex items-center",
    alignmentClass
  );
  const iconEl = iconName && /* @__PURE__ */ jsx(
    Icon,
    {
      name: iconName,
      isDisabled,
      size: iconSize,
      classes: cx(iconClassName, {
        "mr-2": showText && iconAlignment !== "right",
        "ml-2": showText && iconAlignment === "right"
      })
    }
  );
  return /* @__PURE__ */ jsxs(
    "button",
    {
      ref: btnRef,
      id,
      "data-testid": id,
      className: classnames,
      onClick,
      onMouseDown: handleMouseDown,
      onMouseUp,
      onPointerDown: handlePointerDown,
      disabled: isDisabledState,
      ...rest,
      children: [
        iconAlignment !== "right" && iconEl,
        showText && /* @__PURE__ */ jsx("span", { className: cx("pointer-events-none select-none", styles2.content), children: text || children }),
        iconAlignment === "right" && iconEl
      ]
    }
  );
};
function SlidingPanel({
  isOpen = false,
  onClose = () => {
  },
  position = "left",
  title,
  headerComponent,
  children,
  classes = "",
  zIndex = 60,
  pinnable = false,
  defaultPinned = false,
  storageKeyPinned
}) {
  const isVisible = useSignal(false);
  const pinned = useSignal(defaultPinned);
  const ovRef = useRef(null);
  const pnlRef = useRef(null);
  const lastPointerDownOnPanel = useRef(false);
  const isPanelOpen = isOpen || pinned.value;
  useEffect(() => {
    if (storageKeyPinned) {
      const storedPinned = localStorage.getItem(storageKeyPinned);
      if (storedPinned !== null) {
        pinned.value = JSON.parse(storedPinned);
      }
    }
  }, [storageKeyPinned]);
  useEffect(() => {
    if (storageKeyPinned) {
      localStorage.setItem(storageKeyPinned, JSON.stringify(pinned.value));
    }
  }, [pinned.value, storageKeyPinned]);
  useEffect(() => {
    if (isPanelOpen) {
      const timer = setTimeout(() => isVisible.value = true, 10);
      return () => clearTimeout(timer);
    }
  }, [isPanelOpen]);
  const handleClose = () => {
    if (pinned.value) return;
    isVisible.value = false;
    setTimeout(() => onClose(), 300);
  };
  const handleCloseButton = () => {
    pinned.value = false;
    isVisible.value = false;
    setTimeout(() => onClose(), 300);
  };
  const togglePin = () => {
    pinned.value = !pinned.value;
  };
  const handleOverlayInteractionStart = (event) => {
    if (event.target === ovRef.current) {
      lastPointerDownOnPanel.current = false;
    }
  };
  const handleOverlayInteractionEnd = (event) => {
    if (event.target === ovRef.current && !lastPointerDownOnPanel.current) {
      handleClose();
    }
    lastPointerDownOnPanel.current = false;
  };
  const handlePanelInteractionStart = () => {
    lastPointerDownOnPanel.current = true;
  };
  const handlePanelInteractionEnd = () => {
    lastPointerDownOnPanel.current = false;
  };
  if (!isPanelOpen) {
    return null;
  }
  const sideClass = position === "left" ? "left-0" : "right-0";
  const translateClosed = position === "left" ? "-translate-x-full" : "translate-x-full";
  return /* @__PURE__ */ jsxs(
    "div",
    {
      class: pinned.value ? "fixed" : "fixed inset-0",
      role: "dialog",
      "aria-modal": pinned.value ? "false" : "true",
      style: `z-index: ${zIndex}; ${pinned.value ? `${sideClass} top-0 bottom-0` : ""}`,
      children: [
        !pinned.value && isOpen && /* @__PURE__ */ jsx(
          "div",
          {
            ref: ovRef,
            class: `absolute inset-0 bg-black/50 transition-opacity duration-300 ${isVisible.value ? "opacity-100" : "opacity-0"}`,
            "data-testid": "sliding-panel-overlay",
            onPointerDown: handleOverlayInteractionStart,
            onPointerUp: handleOverlayInteractionEnd,
            onMouseDown: handleOverlayInteractionStart,
            onMouseUp: handleOverlayInteractionEnd,
            onTouchStart: handleOverlayInteractionStart,
            onTouchEnd: handleOverlayInteractionEnd
          }
        ),
        /* @__PURE__ */ jsx("div", { class: `fixed inset-y-0 ${sideClass} flex max-w-full`, children: /* @__PURE__ */ jsx(
          "div",
          {
            ref: pnlRef,
            class: `relative w-screen max-w-xs transform transition-transform duration-300 ${isVisible.value ? "translate-x-0" : translateClosed}`,
            onPointerDown: handlePanelInteractionStart,
            onPointerUp: handlePanelInteractionEnd,
            onMouseDown: handlePanelInteractionStart,
            onMouseUp: handlePanelInteractionEnd,
            onTouchStart: handlePanelInteractionStart,
            onTouchEnd: handlePanelInteractionEnd,
            children: /* @__PURE__ */ jsxs(
              "div",
              {
                class: `flex h-full flex-col overflow-y-auto overflow-x-hidden bg-panel shadow-xl  ${classes}`,
                "data-testid": "sliding-panel-content",
                children: [
                  /* @__PURE__ */ jsxs("div", { class: "flex items-start p-2 gap-1", children: [
                    headerComponent ? /* @__PURE__ */ jsx("div", { class: "flex-1 flex items-center", children: headerComponent }) : /* @__PURE__ */ jsx("h5", { class: `flex-1 text-lg text-strong`, children: title }),
                    pinnable && /* @__PURE__ */ jsx(
                      Button,
                      {
                        color: "primary",
                        variant: "ghost",
                        iconOnly: true,
                        iconName: pinned.value ? "pin_off" : "pin",
                        onClick: togglePin,
                        className: "hidden md:block",
                        "aria-label": pinned.value ? "Unpin panel" : "Pin panel"
                      }
                    ),
                    /* @__PURE__ */ jsx(
                      Button,
                      {
                        color: "primary",
                        variant: "ghost",
                        size: "s",
                        iconOnly: true,
                        iconName: "close",
                        onClick: handleCloseButton,
                        "aria-label": "Close panel"
                      }
                    )
                  ] }),
                  /* @__PURE__ */ jsx("div", { class: "p-2 flex-1 overflow-y-auto overflow-x-hidden", style: "scrollbar-gutter: stable;", children })
                ]
              }
            )
          }
        ) })
      ]
    }
  );
}
var sliding_panel_default = SlidingPanel;

export { SlidingPanel, sliding_panel_default as default };
//# sourceMappingURL=sliding-panel.js.map
//# sourceMappingURL=sliding-panel.js.map