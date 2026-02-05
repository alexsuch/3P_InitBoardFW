import * as preact from 'preact';
import { ComponentChildren } from 'preact';
import { HTMLAttributes } from 'preact/compat';

interface SlidingPanelProps extends HTMLAttributes<HTMLDivElement> {
    isOpen?: boolean;
    onClose?: () => void;
    position?: 'left' | 'right';
    title?: string;
    headerComponent?: ComponentChildren;
    classes?: string;
    zIndex?: number;
    pinnable?: boolean;
    defaultPinned?: boolean;
    storageKeyPinned?: string;
}
declare function SlidingPanel({ isOpen, onClose, position, title, headerComponent, children, classes, zIndex, pinnable, defaultPinned, storageKeyPinned, }: SlidingPanelProps): preact.JSX.Element | null;

export { SlidingPanel, type SlidingPanelProps, SlidingPanel as default };
