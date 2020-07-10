import { Component, OnInit, Input, Optional } from '@angular/core';
import { Therblig } from '../models/therblig';
import { Agent } from '../models/agent';
import { Thing } from '../models/thing';
import { Destination } from '../models/destination';
import { MatSnackBar } from '@angular/material';
import { AuthrService } from '../services/authr.service';
import { Route, Router } from '@angular/router';
import { getNotSetAgent, getOptAgent, getNotSetThing, getNotSetDestination } from '../models/notset';

@Component({
  selector: 'therblig-item',
  templateUrl: './therblig.component.html',
  styleUrls: ['./therblig.component.scss']
})
export class TherbligComponent implements OnInit {

  @Input() therbligKey: string; // input if therblig
  @Input() preview: boolean = false;
  private _therblig: Therblig;
  private isPrimitive: boolean = true;
  private optAgent:Agent = getOptAgent();
  private notsetAgent:Agent = getNotSetAgent();
  private notsetThing:Thing = getNotSetThing();
  private notsetDestination:Destination = getNotSetDestination();

  goToDetail(event) {
      if (!this.preview && this.authrService.therbligKeys.indexOf(this.therbligKey) > -1) {
          this.authrService.focused = this.therbligKey;
          this.router.navigate(['/plan',{ outlets: { config: ['therbligs',this.therbligKey] } }]);

      } else if (!this.preview && this.authrService.primitiveKeys.indexOf(this.therbligKey) > -1) {
          this.authrService.focused = this.therbligKey;
          this.router.navigate(['/plan',{ outlets: { config: ['therblig-primitives',this.therbligKey] } }]);
      }
  }

  constructor(public snackbar: MatSnackBar,
              public authrService: AuthrService,
              public router: Router) { }

  ngOnInit() {
      if (this.authrService.primitiveKeys.indexOf(this.therbligKey) > -1) {
        this.isPrimitive = true;
        this._therblig = this.authrService.primitives[this.therbligKey];
      } else if (this.authrService.therbligKeys.indexOf(this.therbligKey) > -1) {
        this.isPrimitive = false;
        this._therblig = this.authrService.getTherbligById(this.therbligKey);
      }
  }

  public set therblig(value:any) {
      if (!this.isPrimitive && value !== null) {
        this.authrService.setTherbligById(value,this.therbligKey);
      }
  }

  public get therblig(): any {
      if (!this.isPrimitive) {
        return this.authrService.getTherbligById(this.therbligKey);
      } else {
        return this._therblig
      }

  }
}
