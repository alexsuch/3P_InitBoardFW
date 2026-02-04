import * as preact from 'preact';
import { ComponentChildren } from 'preact';
import { Signal } from '@preact/signals';

interface TabItem {
    label: string;
    value: string;
    content: ComponentChildren;
    disabled?: boolean;
}
interface TabsProps {
    items: TabItem[];
    initialValue?: string;
    value?: string;
    onValueChange?: (value: string) => void;
    valueSignal?: Signal<string>;
    className?: string;
    panelClassName?: string;
}
declare const Tabs: ({ items, initialValue, value, onValueChange, valueSignal, className, panelClassName }: TabsProps) => preact.JSX.Element;

export { type TabItem, Tabs, type TabsProps };
