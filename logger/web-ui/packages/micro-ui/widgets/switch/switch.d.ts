import * as preact from 'preact';
import { HTMLAttributes } from 'preact/compat';
import { I as IInputProps } from '../../widget-types-CevEJd3U.js';
import { Signal } from '@preact/signals';

interface SwitchProps extends Omit<HTMLAttributes<HTMLInputElement>, 'onChange'>, IInputProps {
    label?: string;
    checked?: boolean | number;
    disabled?: boolean;
    id?: string;
    onChange?: (checked: boolean | number) => void;
    className?: string;
    valueSignal?: Signal<boolean | number>;
    treatBooleanAsNumber?: boolean;
    onValidityChange?: (isValid: boolean) => void;
    size?: 'xs' | 's' | 'm' | 'l' | 'xl';
    loading?: boolean;
}
declare const Switch: ({ label, checked, disabled, id, onChange, className, onValueChange, valueSignal, treatBooleanAsNumber, onValidityChange, size, loading, }: SwitchProps) => preact.JSX.Element;

export { Switch, type SwitchProps };
