import { Component, Input, OnInit, OnDestroy  } from '@angular/core';
import { AuthrService }                        from "../services/authr.service";
import { Destination }                         from '../models/destination';
import { enumSelector, hexToRgb, rgbToHex }    from "../utility.module";
import { Cartesian3D }                         from "../models/cartesian-3d";
import { EulerAngle }                          from "../models/eulerAngle";
import { MatSnackBar }                         from '@angular/material';


@Component({
  selector: 'destination',
  templateUrl: './element-destination.component.html',
  styleUrls: ['./element-destination.component.css']
})
export class ElementDestinationComponent implements OnInit {

  @Input() key: string;

  public mode = 'default';
  public backupDestination = null;
  public currentDestination = null;

  constructor(public snackbar: MatSnackBar,
              public authrService:AuthrService) {

  this.currentDestination = this.authrService.getDestinationById(this.key);

  }

  ngOnInit() {
    this.currentDestination = this.authrService.getDestinationById(this.key);
  }

  public get reachable() {
    return this.authrService.getDestinationById(this.key).reachable;
  }

  public set color(event) {
    this.currentDestination.color = hexToRgb(event);
  }

  public get position(): Cartesian3D {
      this.currentDestination.position = this.authrService.getDestinationById(this.key).position;
      return this.currentDestination.position;
  }

  public set position(value:Cartesian3D) {
      this.currentDestination.position = value;
      this.authrService.setDestinationById({position:value},this.key);
  }

  public get orientation(): EulerAngle {
      this.currentDestination.orientation = this.authrService.getDestinationById(this.key).orientation;
      return this.currentDestination.orientation;
  }

  public set orientation(value:EulerAngle) {
      this.currentDestination.orientation = value;
      this.authrService.setDestinationById({orientation:value},this.key);
  }

  public get color(): string {
      let color = this.currentDestination.color;
      return rgbToHex(color.r,color.g,color.b);
  }

  public emitCustomError(message:string) {
    this.snackbar.open(message,null, {duration: 2000});
  }

  public validatePose() {
    console.log('Validating Pose');
    this.authrService.validatePose(this.key);
  }

  enterEditMode() {
    this.backupDestination = JSON.parse(JSON.stringify(this.currentDestination));
    this.currentDestination.movable = true;
    this.authrService.setDestinationById({movable:true},this.key);
    this.mode = 'edit'
  }

  cancelUpdate() {
    this.currentDestination = this.backupDestination;
    this.currentDestination.movable = true;
    this.authrService.setDestinationById({movable:false},this.key);
    this.mode = 'default';
  }

  saveData() {
    console.log(this.currentDestination.name)
    if (this.currentDestination.name == "") {
      this.emitCustomError("Destination's name cannot be empty, please try again.");
      return;
    }
    console.log(this.currentDestination);
    this.currentDestination.movable = false;
    this.authrService.setDestinationById(this.currentDestination,this.key);
    this.mode = 'default';
  }
}
