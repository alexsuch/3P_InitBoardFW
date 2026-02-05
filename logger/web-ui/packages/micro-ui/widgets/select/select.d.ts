import * as preact from 'preact';
import { Signal } from '@preact/signals';
import { I as IInputProps } from '../../widget-types-CevEJd3U.js';

interface ISelectValue {
    name: string;
    value: any;
}
type OptionType = string | number | ISelectValue;
interface SelectProps extends IInputProps {
    options: OptionType[];
    value?: any;
    valueSignal?: Signal<any>;
    onChange?: (newValue: any) => void;
    placeholder?: string;
    disabled?: boolean;
    className?: string;
    title?: string;
    onValidityChange?: (isValid: boolean) => void;
    required?: boolean;
}
declare const Select: ({ options, value, onChange, placeholder, disabled, className, valueSignal, title, onValueChange, onValidityChange, required, }: SelectProps) => preact.JSX.Element;

export { type ISelectValue, Select };
