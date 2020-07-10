import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { TofTableComponent } from './tof-table.component';

describe('TofTableComponent', () => {
  let component: TofTableComponent;
  let fixture: ComponentFixture<TofTableComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ TofTableComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(TofTableComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
