import * as preact from 'preact';
import * as preact_compat from 'preact/compat';
import { HTMLAttributes } from 'preact/compat';
import { Signal } from '@preact/signals';
import { I as IInputProps } from '../../widget-types-CevEJd3U.js';

interface ValidationRes {
    error?: boolean;
    message?: string;
}
interface Validator {
    validate: (value: any) => ValidationRes;
}
interface InputProps extends HTMLAttributes<HTMLInputElement>, IInputProps {
    label?: string;
    name?: string;
    placeholder?: string;
    className?: string;
    id?: string;
    onChange?: any;
    type?: 'text' | 'password' | 'email' | 'number';
    value?: string | number;
    initialValue?: string;
    valueSignal?: Signal;
    min?: number;
    max?: number;
    error?: string;
    hint?: string;
    disabled?: boolean;
    inputTextClasses?: string;
    allowedCharsPattern?: RegExp;
    validators?: Validator[];
    onValidityChange?: (isValid: boolean) => void;
}
declare const Input: preact.FunctionalComponent<preact_compat.PropsWithoutRef<InputProps> & {
    ref?: preact.Ref<HTMLInputElement> | undefined;
}>;

export { Input, type InputProps, type ValidationRes, type Validator };
