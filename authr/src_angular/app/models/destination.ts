import {Cartesian3D} from './cartesian-3d';
import {EulerAngle} from './eulerAngle';
import {Color} from './color';

export interface Destination {
  id:string;
  name:string;
  type:string;
  bgColor:string;
  txtColor:string;
  timestamp:number;
  color:Color;
  useIcon:boolean;
  position:Cartesian3D;
  orientation:EulerAngle;
  movable:boolean;
  reachable:boolean;
  locked:boolean;
}
