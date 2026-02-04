import { useMemo } from 'preact/hooks';
import { useSignal } from '@preact/signals';
import styles from '../../icon.module-K7UO6GPK.module.css';
import { jsxs, jsx } from 'preact/jsx-runtime';
import styles2 from '../../pagination.module-S4KFJ5BT.module.css';

// src/widgets/pagination/pagination.tsx

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

// src/utils/callbacks.ts
var callIf = (fn, ...args) => {
  if (fn) fn(...args);
};
var setSignal = (signal, value) => {
  if (signal) signal.value = value;
};
var ELLIPSIS = "...";
var range = (start, end) => {
  const length = end - start + 1;
  return Array.from({ length }, (_, i) => start + i);
};
var usePaginationRange = ({ count, page, siblings = 1, boundaries = 1 }) => {
  return useMemo(() => {
    const totalPageNumbersToShow = 1 + siblings * 2 + boundaries * 2;
    const totalSlots = totalPageNumbersToShow + 2;
    if (totalSlots >= count) {
      return range(1, count);
    }
    const leftSiblingIndex = Math.max(page - siblings, 1);
    const rightSiblingIndex = Math.min(page + siblings, count);
    const showLeftDots = leftSiblingIndex > boundaries + 1;
    const showRightDots = rightSiblingIndex < count - boundaries;
    const firstPages = range(1, boundaries);
    const lastPages = range(count - boundaries + 1, count);
    if (!showLeftDots && showRightDots) {
      const leftItemCount = 1 + siblings * 2 + boundaries;
      const leftRange = range(1, leftItemCount);
      return [...leftRange, ELLIPSIS, ...lastPages];
    }
    if (showLeftDots && !showRightDots) {
      const rightItemCount = 1 + siblings * 2 + boundaries;
      const rightRange = range(count - rightItemCount + 1, count);
      return [...firstPages, ELLIPSIS, ...rightRange];
    }
    if (showLeftDots && showRightDots) {
      const middleRange = range(leftSiblingIndex, rightSiblingIndex);
      return [...firstPages, ELLIPSIS, ...middleRange, ELLIPSIS, ...lastPages];
    }
    return range(1, count);
  }, [count, page, siblings, boundaries]);
};
var Pagination = ({
  count,
  initialValue,
  value,
  valueSignal,
  onValueChange,
  siblings = 1,
  boundaries = 1,
  className
}) => {
  const isControlled = valueSignal !== void 0 || value !== void 0;
  const localPage = useSignal(initialValue || 1);
  const currentPage = isControlled ? valueSignal?.value || value || 1 : localPage.value;
  const setCurrentPage = (page) => {
    const newPage = Math.max(1, Math.min(page, count));
    if (currentPage === newPage) return;
    if (!isControlled) localPage.value = newPage;
    callIf(onValueChange, newPage);
    setSignal(valueSignal, newPage);
  };
  const paginationRange = usePaginationRange({ count, page: currentPage, siblings, boundaries });
  const onFirst = () => setCurrentPage(1);
  const onPrev = () => setCurrentPage(currentPage - 1);
  const onNext = () => setCurrentPage(currentPage + 1);
  const onLast = () => setCurrentPage(count);
  const onPage = (page) => setCurrentPage(page);
  const isFirst = currentPage === 1;
  const isLast = currentPage === count;
  if (count <= 1) {
    return null;
  }
  return /* @__PURE__ */ jsxs("nav", { className: cx(styles2.nav, "flex items-center gap-1", className), "aria-label": "Pagination", children: [
    /* @__PURE__ */ jsx("button", { type: "button", className: styles2.b, onClick: onFirst, disabled: isFirst, "aria-label": "First Page", children: /* @__PURE__ */ jsx(Icon, { name: "keyboard_double_arrow_right", size: "s", classes: "rotate-180" }) }),
    /* @__PURE__ */ jsx("button", { type: "button", className: styles2.b, onClick: onPrev, disabled: isFirst, "aria-label": "Previous Page", children: /* @__PURE__ */ jsx(Icon, { name: "keyboard_arrow_right", size: "s", classes: "rotate-180" }) }),
    paginationRange.map((page, index) => {
      if (page === ELLIPSIS) {
        return /* @__PURE__ */ jsx("span", { className: cx(styles2.i, "font-bold"), children: "..." }, `ellipsis-${index}`);
      }
      return /* @__PURE__ */ jsx(
        "button",
        {
          type: "button",
          className: cx(styles2.b, { [styles2.a]: page === currentPage }),
          onClick: () => onPage(page),
          "aria-current": page === currentPage ? "page" : void 0,
          "aria-label": `Page ${page}`,
          children: page
        },
        page
      );
    }),
    /* @__PURE__ */ jsx("button", { type: "button", className: styles2.b, onClick: onNext, disabled: isLast, "aria-label": "Next Page", children: /* @__PURE__ */ jsx(Icon, { name: "keyboard_arrow_right", size: "s" }) }),
    /* @__PURE__ */ jsx("button", { type: "button", className: styles2.b, onClick: onLast, disabled: isLast, "aria-label": "Last Page", children: /* @__PURE__ */ jsx(Icon, { name: "keyboard_double_arrow_right", size: "s" }) })
  ] });
};

export { Pagination };
//# sourceMappingURL=pagination.js.map
//# sourceMappingURL=pagination.js.map