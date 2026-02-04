import { ComponentChildren, JSX } from 'preact';

interface CollapsiblePanelProps {
    header?: ComponentChildren;
    headerComponent?: ComponentChildren;
    icon?: string;
    children: ComponentChildren;
    defaultCollapsed?: boolean;
    className?: string;
}
declare const CollapsiblePanel: ({ header, headerComponent, icon, children, defaultCollapsed, className, }: CollapsiblePanelProps) => JSX.Element;

export { CollapsiblePanel, type CollapsiblePanelProps, CollapsiblePanel as default };
