import { Component, OnInit, OnDestroy } from '@angular/core';
import { AuthrService }                 from "../services/authr.service";
import { MatSnackBar }                  from '@angular/material';

@Component({
  selector: 'setup-things',
  templateUrl: './setup-things.component.html',
  styleUrls: ['./setup-things.component.scss']
})
export class SetupThingsComponent {

  constructor(public authrService: AuthrService,
              public snackbar: MatSnackBar) { }
}
