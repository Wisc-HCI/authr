import { Component, Input, OnInit, OnDestroy } from '@angular/core';
import { AuthrService }                        from "../services/authr.service";
import { Thing, ThingType }                    from '../models/thing';
import { enumSelector, hexToRgb, rgbToHex }    from "../utility.module";
import { MatSnackBar }                         from '@angular/material';

@Component({
  selector: 'thing',
  templateUrl: './element-thing.component.html',
  styleUrls: ['./element-thing.component.css']
})
export class ElementThingComponent {

  @Input() key: string;
  public thingTypes = enumSelector(ThingType);
  public colorUpdateFunction: (value:any) => void;
  private icon = "edit";
  private tip = "Edit Mode";
  private showEdit = false;
  private prevName = "";
  private prevType = "";
  private prevSize = "";
  private prevColor = "";
  private thingName = "";
  private thingType = "";
  private thingSize = "";
  private emptyFlag = 1;

  constructor(public snackbar: MatSnackBar,
              public authrService:AuthrService) {}

  public set color(event) {
    this.authrService.setThingById({color:hexToRgb(event),elementSetType:'thing'},this.key);
  }

  public get name(): string {
    if (this.emptyFlag === 1) {
      this.thingName = this.authrService.things[this.key].name;
      this.thingType = this.authrService.things[this.key].type;
      this.thingSize = this.authrService.things[this.key].size;
      this.emptyFlag = 0;
    }
    return this.thingName;
  }

  public set name(event) {
    this.thingName = event;
  }

  public get type(): string {
    if (this.emptyFlag === 1) {
      this.thingName = this.authrService.things[this.key].name;
      this.thingType = this.authrService.things[this.key].type;
      this.thingSize = this.authrService.things[this.key].size;
      this.emptyFlag = 0;
    }
    return this.thingType;
  }

  public set type(event) {
    this.thingType = event;
  }

  public get size(): string {
    if (this.emptyFlag === 1) {
      this.thingName = this.authrService.things[this.key].name;
      this.thingType = this.authrService.things[this.key].type;
      this.thingSize = this.authrService.things[this.key].size;
      this.emptyFlag = 0;
    }
    return this.thingSize;
  }

  public set size(event) {
    this.thingSize = event;
  }

  public get color(): string {
      return rgbToHex(
        this.authrService.things[this.key].color.r,
        this.authrService.things[this.key].color.g,
        this.authrService.things[this.key].color.b
      );
  }

  public valueCheck(value) {
    if (value === null || value === undefined) {
      value = 0;
    }
    return value;
  }

  public emitCustomError(message:string) {
    this.snackbar.open(message,null, {duration: 2000});
  }

  switchMode(flag) {
    if (this.icon === "edit") {
      this.icon = "save";
      this.tip = "Save Change";
      this.showEdit = true;
      this.prevName = this.authrService.things[this.key].name;
      this.prevType = this.authrService.things[this.key].type;
      this.prevSize = this.authrService.things[this.key].size;
      this.prevColor = this.authrService.things[this.key].color;
    } else if (this.icon === "save") {
      if (this.thingName === "") {
        this.emitCustomError("Thing's name cannot be empty, please try again.");
        this.emptyFlag = 1;
        return;
      }
      if (this.thingSize === "") {
        this.emitCustomError("Thing's size cannot be empty, please try again.");
        this.emptyFlag = 1;
        return;
      }
      this.icon = "edit";
      this.tip = "Edit Mode";
      this.showEdit = false;
      if (flag === 1) {
        this.thingName = this.prevName;
        this.thingType = this.prevType;
        this.thingSize = this.prevSize;
        this.authrService.setThingById({color:this.prevColor,elementSetType:'agent'},this.key);
        return;
      }
      this.authrService.setThingById({name:this.thingName,elementSetType:'thing'},this.key);
      this.authrService.setThingById({type:this.thingType,elementSetType:'thing'},this.key);
      this.authrService.setThingById({size:this.valueCheck(parseFloat(this.thingSize)),elementSetType:'thing'}, this.key);
    }
  }
}
