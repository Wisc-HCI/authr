import { Component, OnInit, OnDestroy } from '@angular/core';
import { AuthrService } from "../services/authr.service";
import { MatSnackBar } from '@angular/material';

@Component({
  selector: 'setup-destinations',
  templateUrl: './setup-destinations.component.html',
  styleUrls: ['./setup-destinations.component.scss']
})
export class SetupDestinationsComponent {

  constructor(public authrService: AuthrService,
              public snackbar: MatSnackBar) { }

  public get showTofs(): boolean {
    return Object.keys(this.authrService.tofs).length > 1;
  }
}
