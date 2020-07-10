import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { SetupDestinationsComponent } from './setup-destinations.component';

describe('SetupDestinationsComponent', () => {
  let component: SetupDestinationsComponent;
  let fixture: ComponentFixture<SetupDestinationsComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ SetupDestinationsComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(SetupDestinationsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
