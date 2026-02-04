// src/inject-sprite.ts
var DEFAULT_CONTAINER_ID = "micro-ui-icon-sprite";
var injectIconSprite = (spriteContent, containerId = DEFAULT_CONTAINER_ID) => {
  if (typeof document === "undefined" || !document.body) {
    console.warn("injectIconSprite: document or document.body not available");
    return null;
  }
  const existing = document.getElementById(containerId);
  if (existing) {
    existing.innerHTML = spriteContent;
    return existing;
  }
  const wrapper = document.createElement("div");
  wrapper.id = containerId;
  wrapper.style.display = "none";
  wrapper.innerHTML = spriteContent;
  document.body.prepend(wrapper);
  return wrapper;
};

export { injectIconSprite };
//# sourceMappingURL=inject-sprite.js.map
//# sourceMappingURL=inject-sprite.js.map