import * as preact from 'preact';
import { Signal } from '@preact/signals';
import { I as IInputProps } from '../../widget-types-CevEJd3U.js';

interface PaginationProps extends Omit<IInputProps, 'valueSignal' | 'value' | 'initialValue'> {
    count: number;
    initialValue?: number;
    value?: number;
    valueSignal?: Signal<number>;
    siblings?: number;
    boundaries?: number;
    className?: string;
    onValidityChange?: (isValid: boolean) => void;
}
declare const Pagination: ({ count, initialValue, value, valueSignal, onValueChange, siblings, boundaries, className, }: PaginationProps) => preact.JSX.Element | null;

export { Pagination, type PaginationProps };
