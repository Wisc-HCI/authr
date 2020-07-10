import { Component, OnInit, Input } from '@angular/core';
import { Macro } from '../models/macro';
import { Agent } from '../models/agent';
import { Thing } from '../models/thing';
import { Destination } from '../models/destination';
import { MatSnackBar } from '@angular/material';
import { AuthrService } from '../services/authr.service';
import { CdkDragDrop } from '@angular/cdk/drag-drop';
import { Route, Router } from '@angular/router';
import { getNotSetAgent, getNotSetThing, getNotSetDestination } from '../models/notset';

@Component({
  selector: 'macro-item',
  templateUrl: './macro.component.html',
  styleUrls: ['./macro.component.css']
})
export class MacroComponent implements OnInit {

  @Input() macroKey: string; // input if macro
  public macro: Macro;

  goToDetail(event) {
      if (this.macroKey !== undefined && this.macroKey  !== null) {
          this.authrService.focused = this.macroKey;
          this.router.navigate(['/plan',{ outlets: { config: ['macros',this.macroKey] } }]);

      }
  }

  constructor(public snackbar: MatSnackBar,
              public authrService: AuthrService,
              public router: Router) { }

  ngOnInit() {
      if(this.macroKey !== undefined) {
        this.macro = this.authrService.getMacroById(this.macroKey);
      }
  }

  public get name() {
      return this.authrService.getMacroById(this.macroKey).name
  }

}
