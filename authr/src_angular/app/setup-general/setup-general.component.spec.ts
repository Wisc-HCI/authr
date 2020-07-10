import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { SetupGeneralComponent } from './setup-general.component';

describe('SetupGeneralComponent', () => {
  let component: SetupGeneralComponent;
  let fixture: ComponentFixture<SetupGeneralComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ SetupGeneralComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(SetupGeneralComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
