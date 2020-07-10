import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { SetupThingsComponent } from './setup-things.component';

describe('SetupThingsComponent', () => {
  let component: SetupThingsComponent;
  let fixture: ComponentFixture<SetupThingsComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ SetupThingsComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(SetupThingsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
