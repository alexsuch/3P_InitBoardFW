import * as preact from 'preact';
import { HTMLAttributes } from 'preact/compat';
import { I as IInputProps } from '../../widget-types-CevEJd3U.js';
import { Signal } from '@preact/signals';

interface CheckboxProps extends Omit<HTMLAttributes<HTMLInputElement>, 'onChange'>, IInputProps {
    label?: string;
    checked?: boolean | number;
    disabled?: boolean;
    id?: string;
    onChange?: (checked: boolean | number) => void;
    className?: string;
    valueSignal?: Signal<boolean | number>;
    size?: 'xs' | 's' | 'm' | 'l' | 'xl';
    treatBooleanAsNumber?: boolean;
}
declare const Checkbox: ({ label, checked, disabled, id, onChange, className, onValueChange, valueSignal, size, treatBooleanAsNumber, }: CheckboxProps) => preact.JSX.Element;

export { Checkbox, type CheckboxProps };
