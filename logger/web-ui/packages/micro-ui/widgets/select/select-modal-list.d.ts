import * as preact from 'preact';
import { ISelectValue } from './select.js';
import '@preact/signals';
import '../../widget-types-CevEJd3U.js';

interface SelectModalListProps {
    options: (string | number | ISelectValue)[];
    currentValue: any;
    onSelect: (value: any) => void;
    title?: string;
}
declare const SelectModalList: ({ options, currentValue, onSelect, title }: SelectModalListProps) => preact.JSX.Element;

export { SelectModalList };
