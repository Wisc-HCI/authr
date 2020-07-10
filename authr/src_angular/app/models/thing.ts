import {Color} from './color';

export enum ThingType {
  SPHERE = "sphere",
  CYLINDER = "cylinder",
  CUBE = "cube",
  CONTAINER = "container",
  NOTSET = 'notset'
}

export interface Thing {
  id:string;
  name:string;
  timestamp:number;
  type:ThingType;
  size:number;
  color:Color;
  bgColor:string;
  txtColor:string;
  useIcon:boolean;
}
